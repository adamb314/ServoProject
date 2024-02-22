import cadquery as cq
from cadquery import exporters
import math

eps = 0.001

def createEncDiscSinCosOffset():
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

    disc = (
            baseWorkplane
            .circle(axisDiameter/2)
            .circle(radius - 2 * offset - (1 - shellAndBaseOverlapRatio) * shellThickness)
            .extrude(-baseThickness)
            .moveTo(offset, 0)
            .circle(radius - offset - shellThickness)
            .circle(2 * radius)
            .cutBlind(-baseThickness)
            .moveTo(offset, 0)
            .circle(radius - offset - 2 * shellThickness)
            .circle(radius - offset - shellThickness)
            .extrude(-height + shellThickness)
        )


    cOfMassCompCutout = 3.915 #3.127

    disc = (
            disc.copyWorkplane(baseWorkplane)
            .moveTo(radius - 2 * offset - (1 - shellAndBaseOverlapRatio) * shellThickness - (cOfMassCompCutout - 100) / 2, 0)
            .rect(cOfMassCompCutout + 100, diameter)
            .cutBlind(-height)
        )

    cylinder = (
            baseWorkplane
            .moveTo(offset, 0)
            .circle(radius - offset)
            .circle(radius - offset - shellThickness)
            .extrude(-height)
        )

    log(f'{cq.Shape.centerOfMass(disc.union(cylinder).val()) = }')

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
    temp = temp.union(temp.shell(0.1))
    mount = (
            mount
            .cut(temp.translate((0,0,baseThickness)))
        )

    return disc.union(cylinder), None, mount
    #return disc, cylinder, mount

def ds3225GearLid():
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
    optSensorBlockThickness = 9.6 + 0.6
    optSensorBlockHeight = 6

    optSensorBlockWidth2 = optSensorBlockWidth + 2
    optSensorBlockWidth2Offset = 8
    optSensorBlockWidth2Transition = 1.5

    result = (
            result.copyWorkplane(baseWorkplane)
            .moveTo(*motorCenterPoint)
            .circle(optDiscCutoutDiameter / 2)
            .cutBlind(baseThickness - gearZOffset - minBaseThicknessAtMotor)
            .moveTo(*motorCenterPoint)
            .circle(motorDiameter / 2)
            .extrude(-motorZOffset)
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
            .extrude(-optSensorBlockHeight)

            .faces('-Z').faces('>>Z[1]').workplane()
            .moveTo(*motorCenterPoint)
            .circle(motorDiameter / 2)
            .cutBlind(-optSensorBlockHeight + motorZOffset)
            .moveTo(*motorCenterPoint)
            .circle(optDiscCutoutDiameter / 2)
            .cutBlind(-optSensorBlockHeight)
        )

    optSensorHeight = 2.7 + 0.2 + 0.1
    optSensorWidth = 3.4 + 0.2
    optSensorDepth = 1.5 + 0.2 - 0.5 - 0.4
    optSensorFrameThickness = 0.4
    optSensorThickness = optSensorDepth + 2
    optSnesorMountingAngle = 35

    temp = (
            baseWorkplane
            .rect(optSensorThickness, optSensorWidth)
            .extrude(-optSensorHeight)
            .faces('>X').workplane()
            .moveTo(0, -optSensorHeight / 2)
            .rect(optSensorWidth - optSensorFrameThickness, optSensorHeight)
            .extrude(optSensorThickness)
        ).translate((-motorDiameter / 2 - optSensorThickness / 2 + optSensorDepth,
                0, baseThickness - gearZOffset - minBaseThicknessAtMotor))

    cutoutSolid1 = temp.rotate((0,0,0), (0,0,1), -optSnesorMountingAngle).translate((motorCenterPoint[0], 0, 0))
    cutoutSolid2 = temp.rotate((0,0,0), (0,0,1), optSnesorMountingAngle).translate((motorCenterPoint[0], 0, 0))

    result = result.cut(cutoutSolid1.union(cutoutSolid2))

    temp = result.faces('+Z').faces('<Z')

    result = (
            temp.workplane().add(temp.wires()).toPending()
            .cutBlind(-optSensorBlockHeight + motorZOffset)
        )

    result = (
            result.copyWorkplane(baseWorkplane).workplane(offset=baseThickness - gearZOffset - minBaseThicknessAtMotor)
            .split(keepTop=True, keepBottom=True)
        ).all()

    return result[0], result[1]

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

ds3225GearLidTop = None
ds3225GearLidBottom = None

encDisc = None
encDiscCylinder = None
turningMount = None

ds3225GearLidTop, ds3225GearLidBottom = ds3225GearLid()

motorGlobalPos = (38.4 - (6.7 + 5 / 2) - (6.1 + 6.1 / 2), 0, -1.5)

defaultViewOpt = {"alpha":0.5, "color": (255, 173, 0)}

motor = createMotor().translate(motorGlobalPos)
show_object(motor, name='motor', options={"alpha":0.5, "color": (0, 127, 255)})

encDisc, encDiscCylinder, turningMount = createEncDiscSinCosOffset()

if ds3225GearLidTop:
    exporters.export(ds3225GearLidTop, 'ds3225GearLidTop.stl')
    show_object(ds3225GearLidTop, name='ds3225GearLidTop', options=defaultViewOpt)

if ds3225GearLidBottom:
    exporters.export(ds3225GearLidBottom, 'ds3225GearLidBottom.stl')
    show_object(ds3225GearLidBottom, name='ds3225GearLidBottom', options=defaultViewOpt)

encDiscViewOpt = {"alpha":0.5, "color": (240, 240, 240)}

if encDisc:
    encDisc = encDisc.rotate((0, 0, 0), (0, 0, 1), 90).translate((motorGlobalPos[0], motorGlobalPos[1], motorGlobalPos[2] + 2.7))
    exporters.export(encDisc, 'ds3225EncDisc.stl')
    show_object(encDisc, name='encDisc', options=encDiscViewOpt)
if encDiscCylinder:
    encDiscCylinder = encDiscCylinder.rotate((0, 0, 0), (0, 0, 1), 90).translate((motorGlobalPos[0], motorGlobalPos[1], motorGlobalPos[2] + 2.7))
    exporters.export(encDiscCylinder, 'ds3225EncDiscCylinder.stl')
    show_object(encDiscCylinder, name='encDiscCylinder', options=encDiscViewOpt)
if turningMount:
    turningMount = turningMount.rotate((0, 0, 0), (0, 0, 1), 90).translate((motorGlobalPos[0], motorGlobalPos[1], motorGlobalPos[2]))
    exporters.export(turningMount, 'ds3225EncDiscTruningMount.stl')
    show_object(turningMount, name='turningMount', options=encDiscViewOpt)
