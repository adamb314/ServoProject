import cadquery as cq
from cadquery import exporters
import math
from scipy.optimize import minimize

eps = 0.001

def createEncDisc():
    diameter = 13
    offset = 1.0
    height = 2.4
    baseThickness = height - 1.4
    shellThickness = 0.5
    shellAndBaseOverlapRatio = 4.0

    axisDiameter = 1.6

    radius = diameter / 2
    edgeHeight = height - baseThickness

    baseWorkplane = cq.Workplane('XY')

    basicDisc = (
            baseWorkplane
            .circle(axisDiameter/2)
            .circle(radius - 2 * offset - (1 - shellAndBaseOverlapRatio) * shellThickness)
            .extrude(-baseThickness)
            .moveTo(offset, 0)
            .circle(radius - offset - shellThickness)
            .circle(2 * radius)
            .cutBlind(-baseThickness)
        )

    def createDiscWithCOfMassCutout(cutoutSize):
        out = (
                basicDisc.copyWorkplane(baseWorkplane)
                .moveTo(radius - 2 * offset - (1 - shellAndBaseOverlapRatio) * shellThickness - (cutoutSize - 100) / 2, 0)
                .rect(cutoutSize + 100, diameter)
                .cutBlind(-height)
                .copyWorkplane(baseWorkplane)
                .circle(axisDiameter/2)
                .circle(axisDiameter/2 + 2 * shellThickness)
                .extrude(-baseThickness)
            )

        cylinder = (
                baseWorkplane
                .moveTo(offset, 0)
                .circle(radius - offset)
                .circle(radius - offset - shellThickness)
                .extrude(-height)
            )

        out = out.union(cylinder)

        return out

    def calcCenterOfMass(cutoutSize):
        obj = createDiscWithCOfMassCutout(cutoutSize)
        return cq.Shape.centerOfMass(obj.val()).x**2

    res = minimize(calcCenterOfMass, 0.0, method='nelder-mead', options={'xatol': 1e-5, 'disp': True})
    disc = createDiscWithCOfMassCutout(res.x[0])

    log(f'{cq.Shape.centerOfMass(disc.val()) = }')

    mount = (
            baseWorkplane
            .moveTo(offset, 0)
            .circle(axisDiameter / 2)
            .circle(radius - offset - shellThickness - 0.1)
            .extrude(-height)
        )

    temp = (
            disc
            .faces('>Z').workplane()
            .circle(axisDiameter / 2)
            .extrude(-baseThickness)
        )

    mount = (
            mount
            .cut(temp.translate((0,0,baseThickness / 2)))
        )

    return disc, mount

def gearLid():
    length = 38.4
    width = 18.3
    baseThickness = 3.1
    edgeFillet = 1.5

    potAxisDiameter = 4
    potAxisBaseDiameter = 5 + 0.4
    potAxisBaseHeight = 3.4
    potAxisBaseFillet = 0.8
    potZOffset = 1.3

    potHeight = 10.4
    potWidth = 10.2
    potMountBoxThickness = 0.8
    potMountBoxHeight = 6.3
    potNotchWidth = 1.2
    potMountBoxNotchThickness = 0.4
    potMountBoxNotchHight = 0.8

    outputGearBaseDiameter = 7.6
    outputGearBaseHeight = 0.9
    outputGearInnerDiameter = 6
    outputGearInnerAxisHeight = 3.2

    originOffsetFromBoxEdge = 6.7 + 5 / 2

    motorDiameter = 17
    motorCenterOffset = 6.1 + 6.1 / 2
    motorGearHoleDiameter = 4
    minBaseThicknessAtMotor = 0.4

    gearZOffset = 1.2
    gearDiameter = 16.2
    gearCutoutHeight = 2.5

    gearAxisHoleDiameter = 1.6 + 0.2
    gearAxisHoleDepth = 1.2 + 0.4 + 1
    gearAxisHoleOffset = 17.5 + 1.6 / 2
    gearAxisBushingDiameter = gearAxisHoleDiameter + 2 * 0.8
    gearAxisBushingHeight = 0.3


    gear2InnerDiameter = 8.4
    gear2InnerHeight = 3
    gear2AxisHoleDepth = 1.5

    screwHoleDiamater = 2.1
    screwHoleEdgeOffset = 0.7 + 2.1 / 2

    bridgeMargin = 0.2

    baseWorkplane = cq.Workplane('XY')

    boxPosEdgeOffset = length - originOffsetFromBoxEdge

    boxCenterPoint = (length / 2 - originOffsetFromBoxEdge, 0)
    motorCenterPoint = (boxPosEdgeOffset - motorCenterOffset, 0)
    gearCenterPoint = (boxPosEdgeOffset - gearAxisHoleOffset, 0)

    result = (
            baseWorkplane
            .move(*boxCenterPoint)
            .rect(length, width)
            .extrude(baseThickness)
            .edges('|Z').fillet(edgeFillet)
        )

    result = (
            result.copyWorkplane(baseWorkplane)
            .moveTo(*boxCenterPoint)
            .rect(length - 2 * screwHoleEdgeOffset, width - 2 * screwHoleEdgeOffset, forConstruction=True)
            .vertices()
            .circle(screwHoleDiamater / 2)
            .cutThruAll()
        )

    topBaseWorkplane = result.faces('>Z').workplane(origin=(0,0,0))

    result = (
            result.copyWorkplane(topBaseWorkplane)
            .circle(outputGearBaseDiameter / 2)
            .extrude(outputGearBaseHeight)
            .copyWorkplane(topBaseWorkplane)
            .circle(outputGearInnerDiameter / 2)
            .extrude(outputGearInnerAxisHeight)
            .faces('>Z').workplane()
            .circle(potAxisDiameter / 2)
            .cutThruAll()
        )

    result = (
            result.copyWorkplane(baseWorkplane)
            .rect(potHeight + 2 * potMountBoxThickness, potWidth + 2 * potMountBoxThickness)
            .extrude(-potMountBoxHeight)

            .faces('<Z').workplane(origin=(0,0,0))
            .moveTo(0, 0)
            .rect(potHeight + 2 * potMountBoxThickness, potWidth)
            .cutBlind(-potMountBoxHeight + potZOffset)

            .moveTo(potHeight / 2 + potMountBoxThickness - potNotchWidth / 2, 0)
            .rect(potNotchWidth, potWidth)
            .cutBlind(-potMountBoxHeight)

            .copyWorkplane(baseWorkplane).workplane(offset=-potZOffset)
            .circle(potAxisBaseDiameter / 2)
            .cutBlind(potAxisBaseHeight)
            .faces('-Z').faces('>>Z[1]').edges('%CIRCLE').fillet(potAxisBaseFillet)
            .faces('<<Z[2]').edges('<Z').fillet((potAxisBaseDiameter - potAxisDiameter) / 2 - eps)

            .faces('<Z').workplane(origin=(0,0,0))
            .rect(potHeight + 2 * potMountBoxThickness, potWidth + 2 * potMountBoxThickness)
            .extrude(potMountBoxNotchHight)
            .moveTo(0, 0)
            .rect(potHeight + 2 * potMountBoxThickness, potWidth - 2 * potMountBoxNotchThickness)
            .cutBlind(potMountBoxNotchHight)
        )

    result = (
            result.copyWorkplane(topBaseWorkplane)
            .moveTo(*motorCenterPoint)
            .circle(gear2InnerDiameter / 2)
            .extrude(gear2InnerHeight)
            .moveTo(*motorCenterPoint)
            .circle(gearAxisBushingDiameter / 2)
            .extrude(gear2InnerHeight + gearAxisBushingHeight)
            .faces('>Z').workplane(origin=(0,0,0))
            .moveTo(*motorCenterPoint)
            .circle(gearAxisHoleDiameter / 2)
            .cutBlind(-gear2AxisHoleDepth)
        )

    result = (
            result.copyWorkplane(topBaseWorkplane).workplane(offset=-gearZOffset)
            .moveTo(*gearCenterPoint)
            .circle(gearDiameter / 2)
            .cutBlind(gearCutoutHeight)
            .moveTo(*gearCenterPoint)
            .circle(gearAxisBushingDiameter / 2)
            .extrude(gearAxisBushingHeight)
            .workplane(offset=gearZOffset + gearAxisBushingHeight)
            .moveTo(*gearCenterPoint)
            .circle(gearAxisHoleDiameter / 2)
            .cutBlind(-gearAxisHoleDepth)
        )

    temp = result.faces('-Z').faces('>Z').faces('>X')
    p0 = baseWorkplane.plane.toLocalCoords(temp.vertices('<Y').val()).toTuple()
    p1 = baseWorkplane.plane.toLocalCoords(temp.vertices('>Y').val()).toTuple()

    zOffset = p0[2]
    p0 = p0[0:2]
    p1 = p1[0:2]

    result = (
            result.copyWorkplane(topBaseWorkplane)
            .moveTo(*p0)
            .lineTo(*p1)
            .lineTo(gearCenterPoint[0], p1[1])
            .lineTo(gearCenterPoint[0], p0[1])
            .close()
            .cutBlind(10)
            .copyWorkplane(baseWorkplane)
            .moveTo(*motorCenterPoint)
            .circle(motorGearHoleDiameter / 2)
            .cutBlind(zOffset)
        )

    mountingNotchOffset = 11.7
    mountingNotchWidth = 2.4
    mountingNotchLength = 16.5

    result = (
            result.copyWorkplane(topBaseWorkplane)
            .moveTo(boxPosEdgeOffset - mountingNotchOffset - mountingNotchWidth / 2)
            .rect(mountingNotchWidth, mountingNotchLength)
            .cutBlind(-gearZOffset)
        )

    optDiscCutoutDiameter = 15
    motorZOffset = 1.5
    optSensorBlockWidth = 14.4
    optSensorBlockThickness = 9.6 + 1
    optSensorBlockHeight = 6
    optSensorHolderMountingMargin = 0.1

    optSensorBlockWidth2 = optSensorBlockWidth + 2
    optSensorBlockWidth2Offset = 7.2
    optSensorBlockWidth2Transition = 1.5

    optSensorBlockGearAxisCutoutLenght = 4
    optSensorBlockGearAxisCutoutWidth = 8

    optDiscCutoutDepth = baseThickness - gearZOffset - minBaseThicknessAtMotor + motorZOffset

    tab = (
            baseWorkplane
            .rect(2 * 0.8, 1.2)
            .extrude(-optDiscCutoutDepth + bridgeMargin + motorZOffset - 1.6)
        ).translate((motorDiameter / 2,
                0, -bridgeMargin))

    teb1 = tab.rotate((0,0,0), (0,0,1), -45).translate((motorCenterPoint[0], 0, 0))
    teb2 = tab.rotate((0,0,0), (0,0,1), 45).translate((motorCenterPoint[0], 0, 0))

    optSensorHolder = (
            cq.Workplane().copyWorkplane(baseWorkplane).workplane(offset=optDiscCutoutDepth - motorZOffset)
            .moveTo(motorCenterPoint[0], optSensorBlockWidth / 2)
            .lineTo(motorCenterPoint[0] - optSensorBlockWidth2Offset + optSensorBlockWidth2Transition,
                optSensorBlockWidth / 2)
            .lineTo(motorCenterPoint[0] - optSensorBlockWidth2Offset, (optSensorBlockWidth2) / 2)
            .lineTo(motorCenterPoint[0] - optSensorBlockThickness, (optSensorBlockWidth2) / 2)
            .lineTo(motorCenterPoint[0] - optSensorBlockThickness, -(optSensorBlockWidth2) / 2)
            .lineTo(motorCenterPoint[0] - optSensorBlockWidth2Offset, -(optSensorBlockWidth2) / 2)
            .lineTo(motorCenterPoint[0] - optSensorBlockWidth2Offset + optSensorBlockWidth2Transition,
                -optSensorBlockWidth / 2)
            .lineTo(motorCenterPoint[0], -optSensorBlockWidth / 2)
            .close()
            .extrude(-optSensorBlockHeight - (optDiscCutoutDepth - motorZOffset))

            .union(teb1.union(teb2))

            .moveTo(motorCenterPoint[0] - optSensorBlockThickness + optSensorBlockGearAxisCutoutLenght / 2, 0)
            .rect(optSensorBlockGearAxisCutoutLenght, optSensorBlockGearAxisCutoutWidth / 2)
            .extrude(-optDiscCutoutDepth - 2 * optSensorBlockHeight)

            .moveTo(*motorCenterPoint)
            .circle(motorDiameter / 2)
            .cutBlind(- 100)

            .moveTo(*motorCenterPoint)
            .circle(motorDiameter / 2)
            .extrude(-optDiscCutoutDepth)

            .moveTo(*motorCenterPoint)
            .circle(optDiscCutoutDiameter / 2)
            .cutBlind(-optDiscCutoutDepth)

            .moveTo(motorCenterPoint[0] - optSensorBlockThickness + optSensorBlockGearAxisCutoutLenght / 2, 0)
            .rect(optSensorBlockGearAxisCutoutLenght, optSensorBlockGearAxisCutoutWidth / 2)
            .cutBlind(-optDiscCutoutDepth - bridgeMargin)
        )

    result = (
            result.copyWorkplane(baseWorkplane)
            .moveTo(*motorCenterPoint)
            .circle(motorDiameter / 2)
            .extrude(- (optDiscCutoutDepth - motorZOffset))

            .moveTo(*motorCenterPoint)
            .circle(optDiscCutoutDiameter / 2)
            .cutBlind(-optDiscCutoutDepth + motorZOffset)

            .moveTo(*motorCenterPoint)
            .circle(optDiscCutoutDiameter / 2)
            .cutBlind(baseThickness - gearZOffset - minBaseThicknessAtMotor)
        )

    splitWorkplane = baseWorkplane.workplane(offset=baseThickness - gearZOffset - minBaseThicknessAtMotor)

    lidTop, lidBottom = (
            result.copyWorkplane(splitWorkplane)
            .split(keepTop=True, keepBottom=True)
        ).all()

    temp = lidBottom.intersect(optSensorHolder)
    temp = temp.union(temp.shell(optSensorHolderMountingMargin))

    lidBottom = lidBottom.cut(temp)

    optSensorHeight = 2.7 + 0.2 + 0.1
    optSensorWidth = 3.4 + 0.2
    optSensorDepth = 1.5 + 0.2 - 0.5 - 0.4
    optSensorFrameThickness = 0.4
    optSensorThickness = optSensorDepth + 2
    optSnesorMountingAngle = 35

    optSensorCutout = (
            baseWorkplane
            .rect(optSensorThickness, optSensorWidth)
            .extrude(-optSensorHeight)
            .faces('>X').workplane()
            .moveTo(0, -optSensorHeight / 2)
            .rect(optSensorWidth - optSensorFrameThickness, optSensorHeight)
            .extrude(optSensorThickness)
        ).translate((-motorDiameter / 2 - optSensorThickness / 2 + optSensorDepth,
                0, baseThickness - gearZOffset - minBaseThicknessAtMotor))

    cutoutSolid1 = optSensorCutout.rotate((0,0,0), (0,0,1), -optSnesorMountingAngle).translate((motorCenterPoint[0], 0, 0))
    cutoutSolid2 = optSensorCutout.rotate((0,0,0), (0,0,1), optSnesorMountingAngle).translate((motorCenterPoint[0], 0, 0))

    optSensorHolder = optSensorHolder.cut(cutoutSolid1.union(cutoutSolid2))

    tempFaces = optSensorHolder.faces('+Z').faces('>>Z[1]')

    optSensorHolder = (
            tempFaces.workplane().add(tempFaces.wires()).toPending()
            .cutBlind(-optSensorBlockHeight)
        )

    pcbMountLength = 40
    pcbMountWidth = 20
    pcbMountWallThickness = 1.2
    pcbMountHeight = 6.2
    pcbMountNotchThickness = 0.8
    pcbMountNotchDepth = 1.2
    pcbMountMotorNotchThickness = 2.2
    pcbMountMotorNotchHeight = 1.8
    pcbMountCableCutout = 2.5

    pcbMount = (
            cq.Workplane('XY')
            .moveTo(*boxCenterPoint)
            .rect(pcbMountLength, pcbMountWidth)
            .rect(pcbMountLength - 2 * pcbMountWallThickness, pcbMountWidth - 2 * pcbMountWallThickness)
            .extrude(-pcbMountHeight - pcbMountMotorNotchHeight)
            .edges('|Z').fillet(screwHoleEdgeOffset)
        )

    screwHoleVertices = (
            pcbMount
            .moveTo(*boxCenterPoint)
            .rect(length - 2 * screwHoleEdgeOffset, width - 2 * screwHoleEdgeOffset, forConstruction=True)
            .vertices()
        )

    tempCutoutSolid = (
            cq.Workplane('XY')
            .moveTo(*boxCenterPoint)
            .rect(pcbMountLength - 2 * pcbMountNotchThickness, pcbMountWidth - 2 * pcbMountNotchThickness)
            .extrude(-pcbMountNotchDepth)
            .edges('|Z').fillet(screwHoleEdgeOffset - pcbMountNotchThickness)
        )

    motorMountNotch = (
            cq.Workplane('XY').workplane(offset=-pcbMountHeight - pcbMountMotorNotchHeight)
            .moveTo(*boxCenterPoint)
            .rect(pcbMountLength, pcbMountWidth)
            .extrude(pcbMountMotorNotchHeight)
            .edges('|Z').fillet(screwHoleEdgeOffset)

            .moveTo(motorCenterPoint[0] - (pcbMountLength - motorDiameter / 2) / 2, 0)
            .rect(pcbMountLength - motorDiameter / 2, pcbMountWidth - 2 * pcbMountMotorNotchThickness)
            .cutBlind(pcbMountMotorNotchHeight)

            .moveTo(*motorCenterPoint)
            .circle(pcbMountWidth / 2 - pcbMountMotorNotchThickness)
            .cutBlind(pcbMountMotorNotchHeight)
        )

    pcbMount = (
            pcbMount
            .union(motorMountNotch)
            .copyWorkplane(screwHoleVertices).add(screwHoleVertices)
            .circle(screwHoleEdgeOffset)
            .extrude(-pcbMountHeight - pcbMountMotorNotchHeight)
            
            .copyWorkplane(screwHoleVertices).add(screwHoleVertices)
            .circle(screwHoleDiamater / 2)
            .cutThruAll()

            .cut(tempCutoutSolid)
        )

    tempHeight = pcbMountHeight + pcbMountMotorNotchHeight
    pcbMount = (
            pcbMount.faces('<X').workplane()
            .moveTo(0, -tempHeight - pcbMountCableCutout)
            .circle(2 * pcbMountCableCutout)
            .cutBlind(-pcbMountWallThickness)

            .moveTo(0, -tempHeight / 2)
            .rect(0.4, tempHeight)
            .cutBlind(-pcbMountWallThickness)
        )

    textDepth = 0.2

    servoProjectText = (
            cq.Workplane().copyWorkplane(pcbMount.faces('<Y').workplane())
            .text('ServoProject', 4.5, -textDepth)
            .translate((pcbMountLength / 2, 0, -pcbMountHeight / 2 - 0.75))
        )

    byAdamText = (
            cq.Workplane().copyWorkplane(pcbMount.faces('>Y').workplane())
            .text('by', 3.5, -textDepth)
            .translate((pcbMountLength - 4.5, 0, -pcbMountHeight / 2))
            .union(
                cq.Workplane().copyWorkplane(pcbMount.faces('>Y').workplane())
                .text('Adam Bäckström', 4, -textDepth)
                .translate((pcbMountLength / 2 - 1.5, 0, -pcbMountHeight + 1))
            )
        )

    pcbMount = pcbMount.cut(servoProjectText.union(byAdamText))

    return lidTop, lidBottom, optSensorHolder, pcbMount

def createMotor():
    diameter = 17
    height = 23
    bushingDiameter = 6.2
    bushingHeight = 1.7
    gearDiameter = 3.1
    gearHeight = 2.3
    axisDiameter = 1.6
    axisHeight = 1

    baseWorkplane = cq.Workplane('XY')

    result = (
            baseWorkplane.circle(diameter / 2)
            .extrude(-height)
        )

    result = (
            result.copyWorkplane(baseWorkplane)
            .circle(bushingDiameter / 2)
            .extrude(bushingHeight)
            .moveTo(0, 0)
            .circle(axisDiameter / 2)
            .extrude(bushingHeight + axisHeight)
            .faces('>Z').workplane()
            .circle(gearDiameter / 2)
            .extrude(gearHeight)
        )

    return result

gearLidTop = None
gearLidBottom = None
optSensorHolder = None
pcbMount = None

encDisc = None
encDiscCylinder = None
turningMount = None

gearLidTop, gearLidBottom, optSensorHolder, pcbMount = gearLid()

motorGlobalPos = (38.4 - (6.7 + 5 / 2) - (6.1 + 6.1 / 2), 0, -1.5)

defaultViewOpt = {"alpha":0.5, "color": (20, 20, 20)}
encDiscViewOpt = {"alpha":0.5, "color": (250, 250, 250)}

motor = createMotor().translate(motorGlobalPos)
show_object(motor, name='motor', options={"alpha":0.2, "color": (160, 160, 160)})

encDisc, turningMount = createEncDisc()

if gearLidTop:
    exporters.export(gearLidTop, 'gearLidTop.stl')
    show_object(gearLidTop, name='gearLidTop', options=defaultViewOpt)

if gearLidBottom:
    exporters.export(gearLidBottom, 'gearLidBottom.stl')
    show_object(gearLidBottom, name='gearLidBottom', options=defaultViewOpt)

if optSensorHolder:
    exporters.export(optSensorHolder, 'optSensorHolder.stl')
    show_object(optSensorHolder, name='optSensorHolder', options=defaultViewOpt)

if pcbMount:
    pcbMount = pcbMount.translate((0, 0, -18.3))
    exporters.export(pcbMount, 'pcbMount.stl')
    show_object(pcbMount, name='pcbMount', options=defaultViewOpt)

if encDisc:
    encDisc = encDisc.rotate((0, 0, 0), (0, 0, 1), 90).translate((motorGlobalPos[0], motorGlobalPos[1], motorGlobalPos[2] + 2.7))
    exporters.export(encDisc, 'encDisc.stl')
    show_object(encDisc, name='encDisc', options=encDiscViewOpt)
if turningMount:
    turningMount = turningMount.rotate((0, 0, 0), (0, 0, 1), 90).translate((motorGlobalPos[0], motorGlobalPos[1], motorGlobalPos[2]))
    exporters.export(turningMount, 'encDiscTruningMount.stl')
    show_object(turningMount, name='encDiscTruningMount', options=encDiscViewOpt)
