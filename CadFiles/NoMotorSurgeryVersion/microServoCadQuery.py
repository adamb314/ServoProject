import cadquery as cq
from cadquery import exporters
import math

def createEncDiscSinCosOffset():
    diameter = 7.5
    offset = 0.55
    height = 1.6
    baseThickness = height - 0.5
    shellThickness = 0.4

    axisDiameter = 1.1

    radius = diameter / 2
    edgeHeight = height - baseThickness

    result = (
            cq.Workplane('XY')
            .circle(axisDiameter/2)
            .circle(radius - 2 * offset - 0.4 * shellThickness)
            .extrude(baseThickness)
            .moveTo(offset, 0)
            .circle(radius - offset)
            .circle(2 * radius)
            .cutBlind(baseThickness)
        )

    result = (
            result.copyWorkplane(cq.Workplane('XY'))
            .moveTo(offset, 0)
            .circle(radius - offset)
            .circle(radius - offset - shellThickness)
            .extrude(height)
        )

    mount = (
            cq.Workplane('XY')
            .moveTo(offset, 0)
            .circle(axisDiameter / 2)
            .circle(radius - offset - shellThickness)
            .extrude(height)
        )

    return result, mount

def createMicroServoBase(squarePot):
    length = 22.5
    width = 11.9
    height = 10.3
    boxWallThickness = 0.8
    originOffsetFromBoxEdge = 4 + 3.5 / 2

    motorCenterOffset = 2.8 + 4 / 2
    motorGearHoleDiameter = 3.4
    motorWidth = 8
    motorHeight = 10
    motorLength = 12.4
    motorWallThickness = 0.8
    baseThicknessAtMotor = 0.4
    motorAnglingStandoffWidth = 0.2
    motorAnglingStandoffHeight = 4
    motorAnglingStandoffThickness = 1.4

    gearAxisHoleDiameter = 1.2 + 0.0
    gearAxisHoleDepth = 2.2 + 0.3
    gearAxisHoleOffset = 10 + gearAxisHoleDiameter / 2

    if squarePot:
        gearAxisHoleOffset -= 0.05
        motorCenterOffset += 0.20

    screwHoleDiamater = 1.2
    screwHeadRestWidth = 4.8
    screwHeadRestThickness = 1

    baseThickness = 2.6

    potAxisHoleDiameter = 1.8

    baseWorkplane = cq.Workplane('XY')

    boxCenterPoint = (length / 2 - originOffsetFromBoxEdge, 0)
    boxPosEdgeOffset = length - originOffsetFromBoxEdge

    baseBox = (
            baseWorkplane
            .move(*boxCenterPoint)
            .rect(length, width)
            .extrude(height)
            .faces('>Z').shell(-boxWallThickness)
        )

    temp = baseBox.faces('+Z').faces('<Z')

    baseBox = (
            temp.workplane()
            .add(temp.wires()).toPending()
            .extrude(baseThickness - boxWallThickness)
        )

    baseBox = (
            baseBox.copyWorkplane(baseWorkplane)
            .moveTo(boxPosEdgeOffset - motorCenterOffset, 0)
            .circle(motorGearHoleDiameter / 2)
            .cutBlind(height)
            .moveTo(0,0)
            .circle(potAxisHoleDiameter / 2)
            .cutBlind(height)
        )

    baseBox = (
            baseBox.copyWorkplane(baseWorkplane)
            .moveTo(boxPosEdgeOffset - gearAxisHoleOffset, 0)
            .circle(gearAxisHoleDiameter / 2)
            .cutBlind(gearAxisHoleDepth)
        )

    innerBaseWorkplane = baseBox.faces('+Z').faces('<Z').workplane(origin=(0,0,0))

    baseBox = (
            baseBox.copyWorkplane(innerBaseWorkplane)
            .moveTo(boxPosEdgeOffset - (motorCenterOffset + motorWidth / 2) / 2, 0)
            .rect(motorCenterOffset + motorWidth / 2, width - 2 * boxWallThickness)
            .extrude(height - baseThickness)
            .faces('>Z').workplane(origin=(0,0,0))
            .moveTo(boxPosEdgeOffset - motorCenterOffset, 0)
            .rect(motorWidth, motorHeight)
            .cutBlind(-height + baseThicknessAtMotor)
            .copyWorkplane(innerBaseWorkplane)
            .moveTo(boxPosEdgeOffset - motorCenterOffset - motorWidth / 2 - motorWallThickness / 2, 0)
            .rect(motorWallThickness, width - 2 * boxWallThickness)
            .extrude(height - baseThickness)
        )

    bushingsPlate = None

    if squarePot:
        potClipLength = 4.7
        potClipCutoutWidth = 1.0
        wallThicknessAtPotClip = 0.4
        potClipEdgeDepth = 0.4 - 0.2
        potCenterPoint = ((1.0 + 9.25 / 2) - (0.6 + 8.6 / 2), 0)
        potLength = 11.1
        potWidth = 9.6 + 0.4
        potMountHoleEdgeOffset = 1.4
        potMountHoleDiameter = 1.5
        potMountHoleDepth = 1
        potPinCutoutDepth = 0.4
        potPinCutoutHeight = 4
        potBushingHeight = 1.6 - 0.2
        potBushingDiameter = potAxisHoleDiameter + 2 * 0.8

        gearAxisBushingDiameter = gearAxisHoleDiameter + 2 * 0.8
        gearAxisBushingHeight = 0.2

        bushingsPlateThickness = 0.8
        bushingsPlateEdgeMargin = 0.1
        bushingsPlateDepthMargin = 0.3

        baseBox = (
            baseBox.copyWorkplane(innerBaseWorkplane)
            .moveTo(boxPosEdgeOffset - motorCenterOffset - motorWidth / 2 - motorWallThickness
                    + potPinCutoutDepth / 2, 0)
            .rect(potPinCutoutDepth, potWidth)
            .cutBlind(potPinCutoutHeight)
        )

        baseBox = (
                baseBox.copyWorkplane(baseWorkplane)
                .moveTo(boxPosEdgeOffset - gearAxisHoleOffset, 0)
                .circle(gearAxisBushingDiameter / 2)
                .circle(gearAxisHoleDiameter / 2)
                .extrude(-gearAxisBushingHeight)
            )

        baseBox = (
                baseBox.copyWorkplane(baseWorkplane)
                .moveTo(0, -potBushingDiameter)
                .hLine(-potBushingDiameter / 2)
                .vLine(2 * potBushingDiameter)
                .hLine(potBushingDiameter / 2)
                .close()
                .extrude(-potBushingHeight)
                .copyWorkplane(baseWorkplane)
                .circle(potAxisHoleDiameter / 2)
                .cutBlind(-potBushingHeight)
            )

        tempLength = length - boxWallThickness - motorCenterOffset - motorWidth / 2 - motorWallThickness
        tempX = -originOffsetFromBoxEdge + boxWallThickness + tempLength / 2
        baseBox = (
                baseBox.copyWorkplane(innerBaseWorkplane)
                .moveTo(tempX, 0)
                .rect(tempLength, width - 2 * boxWallThickness)
                .extrude(height - baseThickness)
                .copyWorkplane(innerBaseWorkplane)
                .moveTo(tempX, 0)
                .rect(tempLength, potWidth)
                .cutBlind(height - baseThickness)
            )

        temp = width / 2 - wallThicknessAtPotClip - potClipCutoutWidth / 2
        baseBox = (
                baseBox.copyWorkplane(baseWorkplane)
                .moveTo(potCenterPoint[0], temp)
                .rect(potClipLength, potClipCutoutWidth)
                .moveTo(potCenterPoint[0], -temp)
                .rect(potClipLength, potClipCutoutWidth)
                .cutBlind(baseThickness - potClipEdgeDepth)
            )

        temp = potWidth / 2 - potClipCutoutWidth / 2
        baseBox = (
                baseBox.copyWorkplane(baseWorkplane)
                .moveTo(potCenterPoint[0], temp)
                .rect(potClipLength, potClipCutoutWidth)
                .moveTo(potCenterPoint[0], -temp)
                .rect(potClipLength, potClipCutoutWidth)
                .cutBlind(baseThickness)
            )

        baseBox = (
                baseBox.copyWorkplane(innerBaseWorkplane)
                .moveTo(*potCenterPoint)
                .rect(potLength - 2 * potMountHoleEdgeOffset, potWidth - 2 * potMountHoleEdgeOffset, forConstruction=True)
                .vertices()
                .circle(potMountHoleDiameter / 2)
                .cutBlind(-potMountHoleDepth)
            )

        cutOutSolid = (
                cq.Workplane('XY').workplane(offset=-potBushingHeight)
                .moveTo(boxPosEdgeOffset - gearAxisHoleOffset + gearAxisBushingDiameter / 2, -potBushingDiameter)
                .lineTo(-potBushingDiameter / 2, -potBushingDiameter)
                .vLine(2 * potBushingDiameter)
                .lineTo(boxPosEdgeOffset - gearAxisHoleOffset + gearAxisBushingDiameter / 2, potBushingDiameter)
                .close()
                .extrude(potBushingHeight + bushingsPlateThickness)
            )

        bushingsPlate = baseBox.intersect(cutOutSolid)

        temp = cutOutSolid.translate((0,0, bushingsPlateDepthMargin - bushingsPlateEdgeMargin))
        cutOutSolid = cutOutSolid.union(temp)
        temp = cutOutSolid.shell(bushingsPlateEdgeMargin)
        cutOutSolid = cutOutSolid.union(temp)
        baseBox = baseBox.cut(cutOutSolid)
    else:
        bushingPotDiameter = 10 + 0.2
        bushingPotHeight = 3.5 + 0.4
        bushingPotNotchThickness = 0.6
        bushingPotNotchWidth = 1.5 + 0.2

        gearAxisBushingDiameter = gearAxisHoleDiameter + 2 * 1.6
        gearAxisBushingHeight = 0.2

        baseBox = (
                baseBox.copyWorkplane(innerBaseWorkplane)
                .moveTo(-bushingPotDiameter / 2 + bushingPotNotchWidth / 2, 0)
                .rect(bushingPotNotchWidth, width)
                .extrude(bushingPotHeight - baseThickness + bushingPotNotchThickness)
                .copyWorkplane(baseWorkplane)
                .circle(bushingPotDiameter / 2)
                .cutBlind(bushingPotHeight)
                .faces('-Z').faces('<X').edges('>X')
                .chamfer(bushingPotNotchThickness - 0.2, bushingPotNotchWidth - 0.2)
            )

        bushingsPlate = (
                cq.Workplane('XY')
                .moveTo(boxPosEdgeOffset - gearAxisHoleOffset, 0)
                .circle(gearAxisBushingDiameter / 2)
                .circle(gearAxisHoleDiameter / 2)
                .extrude(-gearAxisBushingHeight)
            )

    optSensorHeight = 2.7 + 0.2
    optSensorWidth = 3.4 + 0.2
    optSensorDepth = 1.5 + 0.2 - 0.5

    optDiscMotorSpacerLength = 1.5
    optDiscMotorSpacerWidth = 1
    optDiscMotorSpacerHeight = 1.6 + 0.4 + 0.1


    temp = baseBox.faces('>Z').workplane(origin=(0,0,0))
    posBoxWallThickness = boxPosEdgeOffset - (boxPosEdgeOffset - motorCenterOffset + motorWidth / 2)
    baseBox = (
            baseBox.copyWorkplane(temp)
            .moveTo(boxPosEdgeOffset - motorCenterOffset - motorWidth / 2 + motorAnglingStandoffWidth / 2, 0)
            .rect(motorAnglingStandoffWidth, motorAnglingStandoffHeight)
            .extrude(-motorAnglingStandoffThickness)
            .copyWorkplane(temp).workplane(
                    offset= -height + baseThicknessAtMotor + optDiscMotorSpacerHeight + motorAnglingStandoffThickness)
            .moveTo(boxPosEdgeOffset - posBoxWallThickness - motorAnglingStandoffWidth / 2, 0)
            .rect(motorAnglingStandoffWidth, motorAnglingStandoffHeight)
            .extrude(-motorAnglingStandoffThickness)
        )

    baseBox = (
            baseBox.faces('+Z').faces('<Z').workplane(origin=(0,0,0))
            .moveTo(boxPosEdgeOffset - motorCenterOffset, motorHeight / 2 - optDiscMotorSpacerWidth / 2)
            .rect(optDiscMotorSpacerLength, optDiscMotorSpacerWidth)
            .moveTo(boxPosEdgeOffset - motorCenterOffset, -motorHeight / 2 + optDiscMotorSpacerWidth / 2)
            .rect(optDiscMotorSpacerLength, optDiscMotorSpacerWidth)
            .extrude(optDiscMotorSpacerHeight)
        )

    motorPosEdge = boxPosEdgeOffset - posBoxWallThickness
    motorNegEdge = boxPosEdgeOffset - motorCenterOffset - motorWidth / 2
    baseBox = (
            baseBox.faces('<Y').workplane(origin=(0,0,0))
            .moveTo(motorPosEdge + posBoxWallThickness, 0)
            .vLine((optSensorHeight + posBoxWallThickness))
            .lineTo(motorNegEdge - posBoxWallThickness, (optSensorHeight + posBoxWallThickness))
            .vLine(-(optSensorHeight + posBoxWallThickness))
            .close()
            .extrude(optSensorDepth - (width - motorHeight) / 2)
            .faces('<Y').workplane(origin=(0,0,0))
            .moveTo(motorPosEdge, 0)
            .vLine(optSensorHeight)
            .hLine(-optSensorWidth)
            .vLine(-optSensorHeight)
            .close()
            .moveTo(motorNegEdge, 0)
            .vLine(optSensorHeight)
            .hLine(optSensorWidth)
            .vLine(-optSensorHeight)
            .close()
            .cutBlind(-optSensorDepth)
        )

    baseBox = (
            baseBox.faces('>Z').workplane(origin=(0,0,0))
            .moveTo(-originOffsetFromBoxEdge + screwHeadRestWidth, width / 2)
            .hLine(-screwHeadRestWidth)
            .vLine(-screwHeadRestWidth)
            .close()
            .moveTo(-originOffsetFromBoxEdge + screwHeadRestWidth, -width / 2)
            .hLine(-screwHeadRestWidth)
            .vLine(screwHeadRestWidth)
            .close()
            .extrude(-screwHeadRestThickness)
        )

    tempExtrudeHeight = motorLength - (height - baseThicknessAtMotor - optDiscMotorSpacerHeight)
    baseBox = (
            baseBox.faces('>Z').workplane(origin=(0,0,0))
            .moveTo(boxPosEdgeOffset - motorCenterOffset, 0)
            .rect(motorWidth + 2 * posBoxWallThickness, width)
            .rect(motorWidth, motorHeight)
            .extrude(tempExtrudeHeight)
            .faces('>Z').workplane(origin=(0,0,0))
            .moveTo(boxPosEdgeOffset - motorCenterOffset - motorWidth / 2, 0)
            .rect(motorWidth, width)
            .moveTo(boxPosEdgeOffset - posBoxWallThickness / 2, 0)
            .rect(posBoxWallThickness, width / 4)
            .cutBlind(-tempExtrudeHeight)
            .faces('>Z').edges('<X').chamfer(motorWidth / 4)
            .faces('>Z').edges('<<X[1]').chamfer(width / 8)
        )

    screwHoleEdgeOffset = boxWallThickness + screwHoleDiamater / 2
    baseBox = (
            baseBox.copyWorkplane(baseWorkplane)
            .moveTo(*boxCenterPoint)
            .rect(length - 2 * screwHoleEdgeOffset, width - 2 * screwHoleEdgeOffset, forConstruction=True)
            .vertices()
            .circle(screwHoleDiamater / 2)
            .cutThruAll()
        )

    return baseBox, bushingsPlate

def createMg90sBase():
    return createMicroServoBase(False)

def createSg90Base():
    return createMicroServoBase(True)

mg90sBaseBox = None
mg90sBushings = None

sg90BaseBox = None
sg90Bushings = None

#mg90sBaseBox, mg90sBushings = createMg90sBase()
sg90BaseBox, sg90Bushings = createSg90Base()

encDisc, turningMount = createEncDiscSinCosOffset()

if mg90sBaseBox:
    exporters.export(mg90sBaseBox, 'mg90sBaseBox.stl')
if mg90sBushings:
    exporters.export(mg90sBushings, 'mg90sBaseBushings.stl')
if sg90BaseBox:
    exporters.export(sg90BaseBox, 'sg90BaseBox.stl')
if sg90Bushings:
    exporters.export(sg90Bushings, 'sg90BaseBushings.stl')
if encDisc:
    encDisc = encDisc.rotate((0, 0, 0), (0, 0, 1), 90).translate(((22.5 - (4 + 3.5 / 2)) - (2.8 + 4 / 2), 0, 0.8))
    exporters.export(encDisc, 'mg90sEncDisc.stl')
if turningMount:
    turningMount = turningMount.rotate((0, 0, 0), (0, 0, 1), 90).translate(((22.5 - (4 + 3.5 / 2)) - (2.8 + 4 / 2), 0, 2.8))
    exporters.export(turningMount, 'mg90sEncDiscTruningMount.stl')
