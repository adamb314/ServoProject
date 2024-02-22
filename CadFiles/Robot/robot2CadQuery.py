import cadquery as cq
from cadquery import exporters
import math
import numpy as np

parameters = dict()

runCreateRobot = True

tempViewer = None

def getLength(p0, p1):
    temp = 0
    for v0, v1 in zip(p0, p1):
        temp += (v1 - v0)**2
    return math.sqrt(temp)

def mirrorX(mirrorPos, pos):
    return (2 * mirrorPos - pos[0], pos[1])

def projectToPlane(vertex, workplane, *, keepZ=False, asNumpyArray=True):
    vertices = vertex.vals()
    out = [workplane.plane.toLocalCoords(v).toTuple() for v in vertices]
    if not keepZ:
        out = [v[0:2] for v in out]
    if asNumpyArray:
        out = [np.array(v) for v in out]
    if len(out) == 1:
        return out[0]
    return out

def getArcWithLinesPositions(startPos, arcPos, endPos, radius):
    centerPos = (arcPos[0], arcPos[1] + radius)

    def calcOffset(pos):
        centerLength = getLength(centerPos, pos)
        angle = math.acos(-(centerPos[1] - pos[1]) / centerLength)
        angle += math.acos(radius / centerLength)
        offset = (-radius * math.sin(angle),
                radius * math.cos(angle))
        return offset

    startOffset = calcOffset(startPos)
    endOffset = calcOffset(endPos)

    return ((centerPos[0] + startOffset[0], centerPos[1] + startOffset[1]),
            arcPos,
            (centerPos[0] - endOffset[0], centerPos[1] + endOffset[1]))

def getLinePosBetweenArcs(arc0Pos, arc1Pos, radius):
    centerPos0 = (arc0Pos[0], arc0Pos[1] + radius)
    centerPos1 = (arc1Pos[0], arc1Pos[1] - radius)
    centerLength = getLength(centerPos0, centerPos1)

    angle = math.acos(radius / (centerLength / 2))
    angle -= math.acos((centerPos1[0] - centerPos0[0]) / centerLength)
    return ((centerPos0[0] + radius * math.cos(angle),
                centerPos0[1] - radius * math.sin(angle)),
            (centerPos1[0] - radius * math.cos(angle),
                centerPos1[1] + radius * math.sin(angle)))

def calcArcOpeningOffset(radius, openingWidth=None, openingHeight=None):
    if openingWidth:
        xOffset = openingWidth / 2
        yOffset = math.sqrt(radius**2 - xOffset**2)
        return (xOffset, yOffset)
    else:
        yOffset = openingHeight
        xOffset = math.sqrt(radius**2 - yOffset**2)
        return (xOffset, yOffset)

def createLockingClipProfile(*, width, height, countersunkDepth, keyDepth, keyRadia):
    keyStart = (0.0, 0.0)
    keyPos = ((width/2) / 2, -keyDepth - countersunkDepth)
    keyEndPos = (2*keyPos[0], -countersunkDepth)

    lineBetweenArcsPositions = getLinePosBetweenArcs(keyPos,
            keyEndPos, keyRadia)

    keyArcPositions = getArcWithLinesPositions((0.0, 0.0),
            keyPos,
            lineBetweenArcsPositions[1], keyRadia)

    keyArc2Positions = getArcWithLinesPositions(lineBetweenArcsPositions[0],
            keyEndPos,
            (keyEndPos[0]+1, keyEndPos[1]), keyRadia)

    result = (
            cq.Workplane('XY' )
            .lineTo(*keyArcPositions[0])
            .threePointArc(keyArcPositions[1], keyArcPositions[2])
            .lineTo(*lineBetweenArcsPositions[1])
            .radiusArc(keyArc2Positions[1], keyRadia)
            .radiusArc(mirrorX(width / 2, lineBetweenArcsPositions[1]), keyRadia)
            .lineTo(*mirrorX(width / 2, lineBetweenArcsPositions[0]))
            .threePointArc(mirrorX(width / 2, keyArcPositions[1]), mirrorX(width / 2, keyArcPositions[0]))
            .lineTo(*mirrorX(width / 2, keyStart))
            .lineTo(width, height- countersunkDepth)
            .lineTo(0, height- countersunkDepth)
            .close()
        )
    return result

def createLockingClip(*, diameter, width, height, countersunkDepth, keyDepth, keyRadia, edgeMargin, clipOpeningWidth):
    profile = createLockingClipProfile(
        width=width,
        height=height,
        countersunkDepth=countersunkDepth,
        keyDepth=keyDepth,
        keyRadia=keyRadia)

    radius = diameter / 2
    result = profile.revolve(360, (0, -radius, 0), (1, -radius, 0)).translate((0, radius, 0))

    if edgeMargin != 0.0:
        result = result.faces('<X').workplane().circle(100).cutBlind(-edgeMargin)
        result = result.faces('>X').workplane().circle(100).cutBlind(-edgeMargin)

    def createGroveProfileSolid(*, radius, width, cutDepth, cutRadius):
        grovePos = (width / 2, 0.0 - cutDepth)
        groveArcPositions = getArcWithLinesPositions((0.0, 0.0),
            grovePos,
            (width, 0.0), cutRadius)
        topProfile = (
                cq.Workplane('XY' )
                .moveTo(0.0, 0.0)
                .lineTo(*groveArcPositions[0])
                .threePointArc(groveArcPositions[1], groveArcPositions[2])
                .lineTo(width, 0.0)
                .close()
            )
        topProfileSolid = topProfile.revolve(360, (0, -radius, 0), (1, -radius, 0)).translate((0, radius, 0))
        return topProfileSolid

    result = result.cut(createGroveProfileSolid(
            radius=radius + height - countersunkDepth,
            width=width,
            cutDepth=0.2,
            cutRadius=width / 2))

    if clipOpeningWidth != 0.0:
        tempY = height - countersunkDepth + radius
        tempY2 = radius - countersunkDepth - keyDepth - keyRadia
        cutoutRadius = keyDepth + 2 * keyRadia
        result = (
                result.faces('<X')
                .moveTo(-clipOpeningWidth / 2, tempY)
                .lineTo(-clipOpeningWidth / 2, tempY2 + cutoutRadius)
                .radiusArc((-clipOpeningWidth / 2 - cutoutRadius, tempY2), cutoutRadius)
                .lineTo(clipOpeningWidth / 2 + cutoutRadius, tempY2)
                .radiusArc((clipOpeningWidth / 2, tempY2 + cutoutRadius), cutoutRadius)
                .lineTo(clipOpeningWidth / 2, tempY)
                .close()
                .cutThruAll()
            )

    return result

def createClipAxis(*, diameter, thickness, width, countersunkDepth, keyDepth, keyRadia):
    result = (
            cq.Workplane('YZ')
            .circle(diameter / 2)
            .circle(diameter / 2 - thickness)
            .extrude(width)
        )
    lockingClipCutSolid = createLockingClip(
            diameter=diameter,
            width=2*width,
            height=countersunkDepth + 1,
            countersunkDepth=countersunkDepth,
            keyDepth=keyDepth,
            keyRadia=keyRadia,
            edgeMargin=0.0,
            clipOpeningWidth=0.0)

    result = result.cut(lockingClipCutSolid)

    result = (
            result.faces('>X').workplane()
            .circle(diameter / 2 - countersunkDepth - keyDepth / 3)
            .circle(diameter / 2)
            .cutBlind(-width / 2)
        )

    return result

def createMicroServoHolderAxis(*, diameter, thickness, width, countersunkDepth,
            keyDepth, keyRadia, servoAxisOffset, axisMargin):
    result = createClipAxis(
            diameter=diameter,
            thickness=thickness,
            width=width,
            countersunkDepth=countersunkDepth,
            keyDepth=keyDepth,
            keyRadia=keyRadia)

    radius = diameter / 2

    bridgingHeight = parameters['bridgingHeight']
    servoTopCutout = parameters['microServo']['topCutout']
    servoBottomCutout = parameters['microServo']['bottomCutout']
    servoCutoutWidth = parameters['microServo']['cutoutWidth']
    servoCutoutDepth = parameters['microServo']['cutoutDepth']
    servoMountWidth = servoCutoutWidth + 2 * parameters['microServo']['bottomMountThickness']
    servoMountHeight = parameters['microServo']['mountHeight']
    servoMountDepth = parameters['microServo']['mountDepth']
    servoTopMountHole = parameters['microServo']['topMountHole']
    servoBottomMountHole = parameters['microServo']['bottomMountHole']
    servoMountHoleDiameter = parameters['microServo']['mountHoleDiameter']
    servoMountHoleFillet = parameters['microServo']['mountHoleFillet']

    result = (
            result.faces('<X').workplane()
            .moveTo(0, 0)
            .circle(radius)
            .circle(radius - thickness)
            .extrude(servoAxisOffset - servoMountDepth)
        )

    arcOpeningOffset = calcArcOpeningOffset(radius, openingWidth=servoMountWidth)
    result = (
            result.faces('<X').workplane()
            .moveTo(-arcOpeningOffset[0], -arcOpeningOffset[1])
            .lineTo(-servoMountWidth / 2, -servoMountHeight)
            .lineTo(servoMountWidth / 2, -servoMountHeight)
            .lineTo(arcOpeningOffset[0], -arcOpeningOffset[1])
            .radiusArc((0.0, radius), -radius)
            .radiusArc((-arcOpeningOffset[0], -arcOpeningOffset[1]), -radius)
            .close()
            .moveTo(0, 0)
            .circle(radius - thickness)
            .extrude(servoMountDepth)
        )

    arcOpeningOffset = calcArcOpeningOffset(radius - thickness, openingHeight=servoTopCutout)
    result = (
            result.faces('<X').workplane()
            .moveTo(-arcOpeningOffset[0], arcOpeningOffset[1])
            .radiusArc((arcOpeningOffset[0], arcOpeningOffset[1]), radius - thickness)
            .close()
            .extrude(-servoMountDepth + axisMargin)
        )

    arcOpeningOffset = calcArcOpeningOffset(radius - thickness, openingWidth=servoCutoutWidth)
    result = (
            result.faces('<X').workplane()
            .moveTo(0.0, (radius - thickness))
            .radiusArc((-arcOpeningOffset[0], -arcOpeningOffset[1]), -(radius - thickness))
            .lineTo(-servoCutoutWidth / 2, servoTopCutout)
            .lineTo(servoCutoutWidth / 2, servoTopCutout)
            .lineTo(arcOpeningOffset[0], -arcOpeningOffset[1])
            .radiusArc((0.0, (radius - thickness)), -(radius - thickness))
            .close()
            .extrude(-servoMountDepth + axisMargin + bridgingHeight)
        )


    result = (
            result.faces('<X').workplane()
            .moveTo(-servoCutoutWidth / 2, 0)
            .lineTo(-servoCutoutWidth / 2, -servoBottomCutout)
            .lineTo(servoCutoutWidth / 2, -servoBottomCutout)
            .lineTo(servoCutoutWidth / 2, 0)
            .close()
            .moveTo(0, servoTopMountHole)
            .circle(servoMountHoleDiameter / 2)
            .moveTo(0, -servoBottomMountHole)
            .circle(servoMountHoleDiameter / 2)
            .cutBlind(-servoCutoutDepth)
        )

    result = result.faces('<X').edges('not <Z').edges('<Z').fillet(servoMountHoleFillet)
    result = result.faces('<X').edges('>Z').fillet(servoMountHoleFillet)

    return result

runCreateMicroServoAxisClamp = False
def createMicroServoAxisClamp():
    radius = parameters['microServoAxis']['clamp']['radius']
    height = parameters['microServoAxis']['clamp']['height']
    innerDiameter = parameters['microServoAxis']['clamp']['innerDiameter']
    innerDiameterTaper = parameters['microServoAxis']['clamp']['innerDiameterTaper']
    baseThickness = parameters['microServoAxis']['clamp']['baseThickness']
    screwHoleDiameter = parameters['microServoAxis']['clamp']['screwHoleDiameter']
    openingWidth = parameters['microServoAxis']['clamp']['openingWidth']
    rounding = parameters['microServoAxis']['clamp']['rounding']
    innerFilletRadius = parameters['microServoAxis']['clamp']['innerFilletRadius']
    shellThickness = parameters['microServoAxis']['clamp']['shellThickness']

    xOffset = radius * math.sin(math.pi / 3)
    yOffset = radius * math.cos(math.pi / 3)

    result = (
            cq.Workplane('XY')
            .moveTo(0, radius)
            .lineTo(-xOffset, -yOffset)
            .lineTo(xOffset, -yOffset)
            .close()
            .moveTo(0, 0)
            .circle(screwHoleDiameter / 2)
            .extrude(-height)
            .edges('|Z')
            .fillet(rounding)
        )

    servoAxisRadius = innerDiameter / 2
    arcOpeningOffset = calcArcOpeningOffset(servoAxisRadius, openingWidth=openingWidth)

    cutoutSolid = (
            cq.Workplane('XY')
            .moveTo(-arcOpeningOffset[0], -arcOpeningOffset[1])
            .lineTo(-openingWidth / 2, -radius)
            .lineTo(openingWidth / 2, -radius  )
            .lineTo(arcOpeningOffset[0], -arcOpeningOffset[1])
            .radiusArc((0.0, servoAxisRadius ), -servoAxisRadius )
            .radiusArc((-arcOpeningOffset[0], -arcOpeningOffset[1]), -servoAxisRadius)
            .close()
            .extrude(-(height - baseThickness), taper=-innerDiameterTaper)
            .faces('<Z')
            .edges('|Y')
            .fillet(innerFilletRadius)
        )

    result = result.cut(cutoutSolid)
    result = result.cut(cutoutSolid.rotate((0, 0, 0), (0, 0, 1), 120))
    result = result.cut(cutoutSolid.rotate((0, 0, 0), (0, 0, 1), -120))

    result.faces('<Z').translate((0, 0, height)).tag('axisPlane')
    result.faces('<Y').tag('rotationAxisPlane')

    temp = result.faces('>Z').faces('>Y')
    cutoutSolid = (
            cq.Workplane().copyWorkplane(temp.workplane())
            .add(temp.edges()).toPending()
            .offset2D(-shellThickness).extrude(-(height - baseThickness))
        )

    result = result.cut(cutoutSolid)
    result = result.cut(cutoutSolid.rotate((0, 0, 0), (0, 0, 1), 120))
    result = result.cut(cutoutSolid.rotate((0, 0, 0), (0, 0, 1), -120))

    return result

def createMicroServoBaseAxis(*, diameter, innerDiameter, screwHoleDiameter, width, countersunkDepth,
            keyDepth, keyRadia, servoAxisOffset, clampRadius, clampDepth, screwHeadDiameter):
    radius = diameter / 2

    servoMountDepth = parameters['microServo']['mountDepth']
    screwWallThickness = parameters['microServo']['screwWallThickness']
    bridgingHeight = parameters['bridgingHeight']

    result = createClipAxis(
            diameter=diameter,
            thickness=radius - screwHoleDiameter / 2,
            width=width,
            countersunkDepth=countersunkDepth,
            keyDepth=keyDepth,
            keyRadia=keyRadia)

    result = (
            result.rotate((0, 0, 0), (0, 0, 1), 180)
            .translate((width, 0, 0))
            .faces('<X').workplane()
            .circle(innerDiameter / 2)
            .circle(screwHoleDiameter / 2)
            .extrude(servoAxisOffset - servoMountDepth + width)
        )

    xOffset = clampRadius * math.sin(math.pi / 3)
    yOffset = clampRadius * math.cos(math.pi / 3)

    result = (
            result.faces('<X').workplane()
            .moveTo(0, clampRadius)
            .lineTo(-xOffset, -yOffset)
            .lineTo(xOffset, -yOffset)
            .close()
            .cutBlind(-clampDepth)
        )

    clampDepthXPos = -(servoAxisOffset - servoMountDepth + width) + clampDepth

    result = (
            result.faces('>X').workplane()
            .moveTo(0, 0)
            .circle(screwHeadDiameter / 2)
            .cutBlind(clampDepthXPos - width + screwWallThickness)
        )

    arcOpeningOffset = calcArcOpeningOffset(screwHeadDiameter / 2, openingWidth=screwHoleDiameter)
    result = (
            result.faces('>X').workplane()
            .moveTo(-arcOpeningOffset[1], arcOpeningOffset[0])
            .radiusArc((-arcOpeningOffset[1], -arcOpeningOffset[0]), -screwHeadDiameter / 2)
            .lineTo(arcOpeningOffset[1], -arcOpeningOffset[0])
            .radiusArc((arcOpeningOffset[1], arcOpeningOffset[0]), -screwHeadDiameter / 2)
            .close()
            .cutBlind(clampDepthXPos - width + screwWallThickness - bridgingHeight)
        )

    return result

def createBrushedServoHolderAxis(*, diameter, thickness, width, countersunkDepth,
            keyDepth, keyRadia, servoAxisOffset, servoMountWidth=None):
    result = createClipAxis(
            diameter=diameter,
            thickness=thickness,
            width=width,
            countersunkDepth=countersunkDepth,
            keyDepth=keyDepth,
            keyRadia=keyRadia)

    radius = diameter / 2

    bridgingHeight = parameters['bridgingHeight']

    servoTopCutout = parameters['burshedServo']['topCutout']
    servoBottomCutout = parameters['burshedServo']['bottomCutout']
    servoCutoutWidth = parameters['burshedServo']['cutoutWidth']
    servoCutoutDepth = parameters['burshedServo']['cutoutDepth']
    servoMountHeight = parameters['burshedServo']['mountHeight']
    servoMountDepth = parameters['burshedServo']['mountDepth']
    servoMountThickness = parameters['burshedServoAxis']['mountThickness']
    servoTopMountHole = parameters['burshedServo']['topMountHole']
    servoBottomMountHole = parameters['burshedServo']['bottomMountHole']
    servoMountHoleWidth = parameters['burshedServo']['mountHoleWidth']
    servoMountHoleDiameter = parameters['burshedServo']['mountHoleDiameter']
    servoMountHoleDepth = parameters['burshedServo']['mountHoleDepth']
    servoTabWidth = parameters['burshedServo']['tabWidth']
    servoTabDepth = parameters['burshedServo']['tabDepth']

    if not servoMountWidth:
        servoMountWidth = servoCutoutWidth + 2 * parameters['burshedServo']['bottomMountThickness']

    result = (
            result.faces('<X').workplane()
            .moveTo(0, 0)
            .circle(radius)
            .circle(radius - thickness)
            .extrude(servoAxisOffset + servoMountDepth)
        )

    arcOpeningOffset = calcArcOpeningOffset(radius, openingWidth=servoMountWidth)
    result = (
            result.faces('<X').workplane()
            .moveTo(-arcOpeningOffset[0], -arcOpeningOffset[1])
            .lineTo(-servoMountWidth / 2, -servoMountHeight)
            .lineTo(servoMountWidth / 2, -servoMountHeight)
            .lineTo(arcOpeningOffset[0], -arcOpeningOffset[1])
            .radiusArc((0.0, radius), -radius)
            .radiusArc((-arcOpeningOffset[0], -arcOpeningOffset[1]), -radius)
            .close()
            .moveTo(0, 0)
            .circle(radius - thickness)
            .extrude(-servoMountThickness)
        )


    arcOpeningOffset = calcArcOpeningOffset(radius - thickness, openingHeight=servoTopCutout)
    result = (
            result.faces('<X').workplane()
            .moveTo(-arcOpeningOffset[0], arcOpeningOffset[1])
            .radiusArc((arcOpeningOffset[0], arcOpeningOffset[1]), radius - thickness)
            .close()
            .extrude(-servoCutoutDepth)
        )


    arcOpeningOffset = calcArcOpeningOffset(radius - thickness, openingWidth=servoCutoutWidth)
    result = (
            result.faces('<X').workplane()
            .moveTo(0.0, (radius - thickness))
            .radiusArc((-arcOpeningOffset[0], -arcOpeningOffset[1]), -(radius - thickness))
            .lineTo(-servoCutoutWidth / 2, servoTopCutout)
            .lineTo(servoCutoutWidth / 2, servoTopCutout)
            .lineTo(arcOpeningOffset[0], -arcOpeningOffset[1])
            .radiusArc((0.0, (radius - thickness)), -(radius - thickness))
            .close()
            .extrude(-servoCutoutDepth + bridgingHeight)
        )

    result = (
            result.faces('<X').workplane()
            .moveTo(-servoCutoutWidth / 2, 0)
            .lineTo(-servoCutoutWidth / 2, -servoBottomCutout)
            .lineTo(servoCutoutWidth / 2, -servoBottomCutout)
            .lineTo(servoCutoutWidth / 2, 0)
            .close()
            .cutBlind(-servoCutoutDepth)
        )

    result = (
            result.faces('<X').workplane()
            .moveTo(-servoMountHoleWidth / 2, servoTopMountHole)
            .circle(servoMountHoleDiameter / 2)
            .moveTo(servoMountHoleWidth / 2, servoTopMountHole)
            .circle(servoMountHoleDiameter / 2)
            .moveTo(servoMountHoleWidth / 2, -servoBottomMountHole)
            .circle(servoMountHoleDiameter / 2)
            .moveTo(-servoMountHoleWidth / 2, -servoBottomMountHole)
            .circle(servoMountHoleDiameter / 2)
            .cutBlind(-servoMountHoleDepth)
        )

    result = (
            result.faces('<X').workplane()
            .moveTo(-servoTabWidth / 2, servoTopMountHole + servoMountHoleDiameter / 2)
            .lineTo(-servoTabWidth / 2, -servoBottomMountHole - servoMountHoleDiameter / 2)
            .lineTo(servoTabWidth / 2, -servoBottomMountHole - servoMountHoleDiameter / 2)
            .lineTo(servoTabWidth / 2, servoTopMountHole + servoMountHoleDiameter / 2)
            .close()
            .cutBlind(-servoTabDepth)
        )

    return result

def createBrushedServoBaseAxis(*, diameter, innerDiameter, width, countersunkDepth,
            keyDepth, keyRadia, servoAxisOffset, enableBridging):
    radius = diameter / 2

    bridgingHeight = parameters['bridgingHeight']
    screwWallThickness = parameters['burshedServo']['screwWallThickness']
    screwHoleDiameter = parameters['burshedServo']['screwHoleDiameter']
    screwHeadDiameter = parameters['burshedServo']['screwHeadDiameter']
    secScrewOffset = parameters['burshedServoAxis']['secScrewOffset']

    result = createClipAxis(
            diameter=diameter,
            thickness=radius - screwHoleDiameter / 2,
            width=width,
            countersunkDepth=countersunkDepth,
            keyDepth=keyDepth,
            keyRadia=keyRadia)

    result = (
            result.rotate((0, 0, 0), (0, 0, 1), 180)
            .translate((width, 0, 0))
            .faces('<X').workplane()
            .circle(innerDiameter / 2)
            .circle(screwHoleDiameter / 2)
            .extrude(servoAxisOffset + width)
        )

    result = (
            result.faces('>X').workplane()
            .moveTo(0, 0)
            .circle(screwHeadDiameter / 2)
            .cutBlind(-servoAxisOffset - 2 * width + screwWallThickness)
        )

    if enableBridging:
        arcOpeningOffset = calcArcOpeningOffset(screwHeadDiameter / 2, openingWidth=screwHoleDiameter)
        result = (
                result.faces('>X').workplane()
                .moveTo(-arcOpeningOffset[1], arcOpeningOffset[0])
                .radiusArc((-arcOpeningOffset[1], -arcOpeningOffset[0]), -screwHeadDiameter / 2)
                .lineTo(arcOpeningOffset[1], -arcOpeningOffset[0])
                .radiusArc((arcOpeningOffset[1], arcOpeningOffset[0]), -screwHeadDiameter / 2)
                .close()
                .cutBlind(-servoAxisOffset - 2 * width + screwWallThickness - bridgingHeight)
                .moveTo(-screwHoleDiameter / 2, screwHoleDiameter / 2)
                .lineTo(screwHoleDiameter / 2, screwHoleDiameter / 2)
                .lineTo(screwHoleDiameter / 2, -screwHoleDiameter / 2)
                .lineTo(-screwHoleDiameter / 2, -screwHoleDiameter / 2)
                .close()
                .cutBlind(-servoAxisOffset - 2 * width + screwWallThickness - 2 * bridgingHeight)
            )

    result = (
            result.faces('>X').workplane()
            .moveTo(0, -secScrewOffset)
            .circle(screwHeadDiameter / 2)
            .cutBlind(-servoAxisOffset - 2 * width + screwWallThickness)
        )

    result = (
            result.faces('+X').faces('not >X').faces('<Z').edges()
            .chamfer((screwHeadDiameter - screwHoleDiameter) / 2)
        )

    result = (
            result.faces('>X').workplane()
            .moveTo(0, -secScrewOffset)
            .circle(screwHoleDiameter / 2)
            .cutBlind(-servoAxisOffset - 2 * width)
        )

    result = (
            result.translate((width, 0, 0))
        )

    return result

###########################################################################

runCreateJ0 = False
def createJ0():
    diameter = parameters['burshedServoAxis']['diameter']
    thickness = parameters['burshedServoAxis']['thickness']
    servoAxisOffset = parameters['burshedServoAxis']['servoAxisOffset']
    keyWidth = parameters['burshedServoAxis']['clip']['width'] / 2
    countersunkDepth = parameters['burshedServoAxis']['clip']['countersunkDepth']
    clipCutoutMargin = parameters['burshedServoAxis']['clip']['cutoutMargin']
    keyDepth = parameters['burshedServoAxis']['clip']['keyDepth']
    keyRadia = parameters['burshedServoAxis']['clip']['keyRadia']
    innerDiameter = diameter - 2 * thickness - 2 * parameters['burshedServoAxis']['innerAxisMargin']

    servoMountDepth = parameters['burshedServo']['mountDepth']
    mountHoleDiameter = parameters['burshedServo']['mountHoleDiameter']

    j2AxisXOffset = parameters['j1']['j2AxisXOffset']
    j2AxisYOffset = parameters['j1']['j2AxisYOffset']
    servoCutoutHight = parameters['j1']['servoCutoutHight']
    j2BaseAxisExtraThinckness = parameters['j1']['j2BaseAxisExtraThinckness']

    radius = diameter / 2

    result = createBrushedServoBaseAxis(
            diameter=diameter,
            innerDiameter=innerDiameter,
            width=keyWidth,
            countersunkDepth=countersunkDepth,
            keyDepth=keyDepth,
            keyRadia=keyRadia,
            servoAxisOffset=servoAxisOffset,
            enableBridging=True
        )

    result = (
            result
            .rotate((0, 0, 0), (0, 1, 0), 90)
        )

    result.faces('+Z').faces('not >Z').faces('>Z').tag('fromAxisPlane')
    result.faces('-Y').faces('<Y').tag('rotationAxisPlane')
    result.faces(tag='fromAxisPlane').translate((0, 0, servoAxisOffset + keyWidth)).tag('servoJ1AxisPlane')

    mountHoleWidth = 40.6
    mountHoleHeight = 60.6
    mountHoleDepth = 6
    mountHolePadWidth = 16
    mountHolePadHeight = 8

    tempFace = result.faces('<Z')
    result = (
            tempFace.workplane().add(tempFace.wires()).toPending()
            .extrude(mountHoleDepth)
        )

    mountingPad = (
            cq.Workplane().copyWorkplane(tempFace.workplane(origin=(0,0,0)))
            .circle(diameter / 2)
            .circle(diameter / 2 + mountHolePadHeight)
            .extrude(mountHoleDepth)
            .rect(mountHoleWidth, mountHoleHeight, forConstruction=True)
            .vertices()
            .circle(mountHoleDiameter / 2)
            .cutThruAll()
        )

    result = result.union(mountingPad)

    return result

runCreateJ1 = False
def createJ1():
    diameter = parameters['burshedServoAxis']['diameter']
    thickness = parameters['burshedServoAxis']['thickness']
    servoAxisOffset = parameters['burshedServoAxis']['servoAxisOffset']
    keyWidth = parameters['burshedServoAxis']['clip']['width'] / 2
    countersunkDepth = parameters['burshedServoAxis']['clip']['countersunkDepth']
    clipCutoutMargin = parameters['burshedServoAxis']['clip']['cutoutMargin']
    keyDepth = parameters['burshedServoAxis']['clip']['keyDepth']
    keyRadia = parameters['burshedServoAxis']['clip']['keyRadia']
    innerDiameter = diameter - 2 * thickness - 2 * parameters['burshedServoAxis']['innerAxisMargin']

    servoMountDepth = parameters['burshedServo']['mountDepth']

    j2AxisXOffset = parameters['j1']['j2AxisXOffset']
    j2AxisYOffset = parameters['j1']['j2AxisYOffset']
    servoCutoutHight = parameters['j1']['servoCutoutHight']
    j2BaseAxisExtraThinckness = parameters['j1']['j2BaseAxisExtraThinckness']

    radius = diameter / 2

    result = createBrushedServoHolderAxis(
            diameter=diameter,
            thickness=thickness,
            width=keyWidth,
            countersunkDepth=countersunkDepth,
            keyDepth=keyDepth,
            keyRadia=keyRadia,
            servoAxisOffset=servoAxisOffset
        )

    result.faces('+X').faces('>X').tag('toAxisPlane')
    result.faces('-Z').faces('<Z').tag('rotationAxisPlane')
    result.faces('-Y').faces('<Y').tag('rotationOrthAxisPlane')

    result2 = createBrushedServoBaseAxis(
            diameter=diameter,
            innerDiameter=innerDiameter,
            width=keyWidth,
            countersunkDepth=countersunkDepth,
            keyDepth=keyDepth,
            keyRadia=keyRadia,
            servoAxisOffset=servoAxisOffset,
            enableBridging=False
        )

    tempHeight = -(-j2AxisXOffset + j2BaseAxisExtraThinckness + 2 * keyWidth)

    result2 = (
            result2.faces('>X').wires().toPending()
            .extrude(j2BaseAxisExtraThinckness)
        )

    result2 = (
            result2.faces('>X').workplane()
            .circle(radius)
            .extrude(tempHeight)
            .faces('>X').workplane()
            .moveTo(-radius, -radius)
            .lineTo(-radius, servoCutoutHight)
            .lineTo(radius, servoCutoutHight)
            .lineTo(radius, -radius)
            .close()
            .cutBlind(-tempHeight)
        )

    temp = j2AxisXOffset

    result2 = (
            result2
            .rotate((0, 0, 0), (0, 0, 1), -90)
            .translate((-radius - j2AxisYOffset, temp, 0))
        )

    result2.faces('+Y').faces('not >Y').faces('>Y').tag('fromAxisPlane')
    result2.faces(tag='fromAxisPlane').translate((0, servoAxisOffset + keyWidth, 0)).tag('servoJ2AxisPlane')

    tempHeight = 2 * radius + j2AxisYOffset - (servoAxisOffset + servoMountDepth)

    tempX = -j2AxisXOffset + j2BaseAxisExtraThinckness + 2 * keyWidth

    result = (
            result.faces('<X').workplane()
            .circle(radius)
            .extrude(tempHeight)
            .faces('<X').workplane()
            .moveTo(tempX, -radius)
            .lineTo(tempX, servoCutoutHight)
            .lineTo(-tempX - 8, servoCutoutHight)
            .threePointArc((-tempX - 8 + 0.1, servoCutoutHight - 0.001), (radius, -8))
            .lineTo(radius, -radius)
            .close()
            .cutBlind(-tempHeight)
        )

    result = (
            result.faces('- Y').faces('>Y').workplane(origin=(-radius - j2AxisYOffset, 0, 0))
            .circle(radius)
            .cutBlind(-100)
        )

    result = result.union(result2)

    result = (
            result.faces('>Y').workplane(origin=(-radius - j2AxisYOffset, 0, 0))
            .moveTo(0, -radius)
            .radiusArc((0, radius), -radius)
            .lineTo(radius + 1, radius)
            .lineTo(radius + 1, -radius)
            .close()
            .cutThruAll()
        )

    result = (
            result.faces('- Z').faces('>Z')
        )

    def projectToYX(tuple3D):
        return (tuple3D[0], -tuple3D[1])

    vertex0 = projectToYX(
            result.vertices('>Y').vertices('<X').val().toTuple()
        )

    vertex2 = projectToYX(
            result.vertices('<Y').vertices('>X').val().toTuple()
        )

    roundingRadius = parameters['j1']['roundingRadius']
    angle = parameters['j1']['roundingAngle'] / 180 * math.pi

    vertex1 = (vertex0[0] + roundingRadius * (1+math.sin(angle)), vertex0[1] + roundingRadius * math.cos(angle))
    vertex2 = (vertex2[0] + roundingRadius, radius)

    result = (
            result.workplane(origin=(0,0,0))
            .moveTo(*vertex0)
            .threePointArc((vertex0[0] + 0.1, vertex0[1] + 0.001), vertex1)
            .threePointArc((vertex1[0] + 0.1 * math.sin(angle), vertex1[1] + 0.1 * math.cos(angle)), vertex2)
            .lineTo(2 * vertex0[0], vertex2[1])
            .lineTo(2 * vertex0[0], vertex0[1])
            .close()
            .cutThruAll()
        )

    result = (
            result.faces('>Y').workplane(origin=(-radius - j2AxisYOffset, 0, 0), offset=radius)
            .circle(radius + clipCutoutMargin)
            .circle(radius)
            .cutBlind(-2 * keyWidth - servoAxisOffset - radius)
        )

    return result

runCreateJ2 = False
def createJ2(asOneShape=True):
    diameter = parameters['burshedServoAxis']['diameter']
    thickness = parameters['burshedServoAxis']['thickness']
    servoAxisOffset = parameters['burshedServoAxis']['servoAxisOffset']
    keyWidth = parameters['burshedServoAxis']['clip']['width'] / 2
    countersunkDepth = parameters['burshedServoAxis']['clip']['countersunkDepth']
    clipCutoutMargin = parameters['burshedServoAxis']['clip']['cutoutMargin']
    keyDepth = parameters['burshedServoAxis']['clip']['keyDepth']
    keyRadia = parameters['burshedServoAxis']['clip']['keyRadia']
    innerDiameter = diameter - 2 * thickness - 2 * parameters['burshedServoAxis']['innerAxisMargin']

    servoMountDepth = parameters['burshedServo']['mountDepth']

    j2AxisXOffset = parameters['j1']['j2AxisXOffset']
    j2AxisYOffset = parameters['j1']['j2AxisYOffset']
    servoCutoutHight = parameters['j1']['servoCutoutHight']
    j2BaseAxisExtraThinckness = parameters['j1']['j2BaseAxisExtraThinckness']

    j2Length = parameters['j2']['length']
    servo2Angle = parameters['j2']['servo2Angle']
    servo3Angle = parameters['j2']['servo3Angle']
    servo2SideWidth = parameters['j2']['servo2SideWidth']
    servo3SideWidth = parameters['j2']['servo3SideWidth']
    servo3SideOffset = parameters['j2']['servo3SideOffset']
    servoCutoutTop = parameters['j2']['servoCutoutTop']
    servoCutoutBottom = parameters['j2']['servoCutoutBottom']
    servoCutoutWidth = parameters['j2']['servoCutoutWidth']

    roundingHeight = parameters['j2']['roundingHeight']
    roundingRadius = parameters['j2']['roundingRadius']
    roundingStartOffset = parameters['j2']['roundingStartOffset']

    topBottomKeyLength = parameters['j2']['topBottomKey']['length']
    topBottomKeyThickness = parameters['j2']['topBottomKey']['thickness']
    topBottomKeyMargin = parameters['j2']['topBottomKey']['margin']

    radius = diameter / 2

    result = createBrushedServoHolderAxis(
            diameter=diameter,
            thickness=thickness,
            width=keyWidth,
            countersunkDepth=countersunkDepth,
            keyDepth=keyDepth,
            keyRadia=keyRadia,
            servoAxisOffset=servoAxisOffset,
            servoMountWidth=servo2SideWidth
        )

    tempHeight = servoAxisOffset + parameters['burshedServo']['mountDepth']
    result = (
            result.faces('<X')
            .wires('not <Z')
            .wires('not <Z')
            .wires('not >Z')
            .toPending()
            .translate((-roundingHeight, 0, 0)).toPending()
            .loft()
            .faces('<X').workplane(origin=(0,0,0))
            .moveTo(-servoCutoutWidth / 2, servoCutoutBottom)
            .lineTo(servoCutoutWidth / 2, servoCutoutBottom)
            .lineTo(servoCutoutWidth / 2, -servoCutoutTop)
            .lineTo(-servoCutoutWidth / 2, -servoCutoutTop)
            .close()
            .cutBlind(-roundingHeight)
            .faces('-Y').faces('<Y').workplane(origin=(0,0,0))
            .moveTo(-tempHeight + roundingStartOffset, radius)
            .radiusArc((-roundingHeight - tempHeight, -radius), -roundingRadius)
            .lineTo(-roundingHeight - tempHeight, radius)
            .close()
            .cutThruAll()
        )

    result = (
            result
            .rotate((0, 0, 0), (1, 0, 0), 180)
        )

    result.faces('+X').faces('>X').tag('toAxisPlane')
    result.faces('-Z').faces('>Z').tag('rotationAxisPlane')
    result.faces('+Y').faces('>Y').tag('rotationOrthAxisPlane')

    topBottomKeyBaseFace = result.faces('>Z')

    result2 = createBrushedServoHolderAxis(
            diameter=diameter,
            thickness=thickness,
            width=keyWidth,
            countersunkDepth=countersunkDepth,
            keyDepth=keyDepth,
            keyRadia=keyRadia,
            servoAxisOffset=servoAxisOffset,
            servoMountWidth=servo3SideWidth
        )

    tempHeight = servoAxisOffset + parameters['burshedServo']['mountDepth']
    result2 = (
            result2.faces('<X')
            .wires('not <Z')
            .wires('not <Z')
            .wires('not >Z')
            .toPending()
            .translate((-roundingHeight, 0, 0)).toPending()
            .loft()
            .faces('<X').workplane(origin=(0,0,0))
            .moveTo(-servoCutoutWidth / 2, servoCutoutBottom)
            .lineTo(servoCutoutWidth / 2, servoCutoutBottom)
            .lineTo(servoCutoutWidth / 2, -servoCutoutTop)
            .lineTo(-servoCutoutWidth / 2, -servoCutoutTop)
            .close()
            .cutBlind(-roundingHeight)
            .faces('-Y').faces('<Y').workplane(origin=(0,0,0))
            .moveTo(-tempHeight + roundingStartOffset, radius)
            .radiusArc((-roundingHeight - tempHeight, -radius), -roundingRadius)
            .lineTo(-roundingHeight - tempHeight, radius)
            .close()
            .cutThruAll()
        )

    result2 = (
            result2
            .rotate((0, 0, 0), (1, 0, 0), -servo3Angle)
            .translate((0, math.sin(math.pi * servo2Angle / 180) * j2Length, 
                math.cos(math.pi * servo2Angle / 180) * j2Length))
        )

    result2.faces('+X').faces('>X').tag('fromAxisPlane')

    temp = result2.faces('not <Z').faces('<Z')
    result2 = (
            temp.workplane().add(temp.wires()).toPending()
            .extrude(servo3SideOffset)
        )

    temp = result.faces('>Z')
    result2 = (
            result2.faces('<Z').wires().toPending()
            .copyWorkplane(temp.workplane()).add(temp.wires()).toPending().loft()
        )

    result = result.union(result2)

    bendLength = 16

    def extrudeFace(face, offset):
        wires = face.wires().val()
        return (
                cq.Workplane().copyWorkplane(face)
                .add(wires).toPending()
                .translate((offset, 0, 0)).toPending().loft()
            )

    faceToExt = result.faces('+X').faces('not >X').faces('>X')
    cutoutSolid = extrudeFace(faceToExt, -bendLength)

    tempFace = cutoutSolid.faces('>X')
    tempWorkplane = cq.Workplane('XZ')
    vx0 = projectToPlane(tempFace.vertices('<Z'), tempWorkplane)[0]
    vx1 = projectToPlane(tempFace.vertices('>Z'), tempWorkplane)
    vecBend = np.array((-bendLength, 0))

    cutoutSolid = (
            cutoutSolid.copyWorkplane(tempWorkplane)
            .moveTo(*vx0)
            .lineTo(*((vx0 + vx1) / 2 + vecBend))
            .lineTo(*vx1)
            .lineTo(*(vx1 + 2 * vecBend))
            .lineTo(*(vx0 + 2 * vecBend))
            .close()
            .cutThruAll()
        )

    tempFace = cutoutSolid.faces('>Z')
    vx0 = projectToPlane(tempFace.vertices('>Z'), tempFace.workplane())
    vx1 = projectToPlane(tempFace.vertices('<X'), tempFace.workplane())

    temp = vx1 - vx0
    xRatio = temp[0] / temp[1]

    cutoutSolid = (
            tempFace.workplane()
            .moveTo(*vx0)
            .hLine(-bendLength)
            .vLine(-bendLength / xRatio)
            .close()
            .cutThruAll()
        )

    result = (
            result.cut(cutoutSolid)
        )


    faceToExt = result.faces('<X')
    addSolid = extrudeFace(faceToExt, -bendLength)

    tempFace = addSolid.faces('>X')
    tempWorkplane = cq.Workplane('XZ')
    vx0 = projectToPlane(tempFace.vertices('<Z'), tempWorkplane)[0]
    vx1 = projectToPlane(tempFace.vertices('>Z'), tempWorkplane)

    addSolid = (
            addSolid.copyWorkplane(tempWorkplane)
            .moveTo(*vx0)
            .lineTo(*((vx0 + vx1) / 2 + vecBend))
            .lineTo(*vx1)
            .lineTo(*(vx1 + 2 * vecBend))
            .lineTo(*(vx0 + 2 * vecBend))
            .close()
            .cutThruAll()
        )

    tempFace = addSolid.faces('not >Z').faces('not >Z').faces('>Z')
    vx0 = projectToPlane(tempFace.vertices('>Z'), tempFace.workplane())
    vx1 = projectToPlane(tempFace.vertices('<X'), tempFace.workplane())

    temp = vx1 - vx0
    xRatio = temp[0] / temp[1]

    addSolid = (
            tempFace.workplane()
            .moveTo(*vx0)
            .hLine(-bendLength)
            .vLine(-bendLength / xRatio)
            .close()
            .cutThruAll()
        )

    result = result.union(addSolid)

    result = (
            result.copyWorkplane(topBottomKeyBaseFace.workplane())
            .split(keepTop=True, keepBottom=True)
        )

    resultTop = result.first()
    resultBottom = result.last()

    topBottomKey = (
            resultTop.faces('<Z').workplane(offset=-topBottomKeyLength)
            .split(keepTop=True)
            .faces('>Z or <Y')
        )

    topBottomKey = topBottomKey.shell(-topBottomKeyThickness)

    resultBottom = resultBottom.union(topBottomKey)

    temp = topBottomKey.translate((0,0,0)).union(topBottomKey.shell(topBottomKeyMargin))
    resultTop = resultTop.cut(temp)

    if asOneShape:
        return resultTop.union(resultBottom)

    return resultTop, resultBottom

runCreateJ3 = False
def createJ3():
    j2Diameter = parameters['burshedServoAxis']['diameter']
    thickness = parameters['burshedServoAxis']['thickness']
    j2ServoAxisOffset = parameters['burshedServoAxis']['servoAxisOffset']
    j2KeyWidth = parameters['burshedServoAxis']['clip']['width'] / 2
    countersunkDepth = parameters['burshedServoAxis']['clip']['countersunkDepth']
    clipCutoutMargin = parameters['burshedServoAxis']['clip']['cutoutMargin']
    keyDepth = parameters['burshedServoAxis']['clip']['keyDepth']
    keyRadia = parameters['burshedServoAxis']['clip']['keyRadia']
    innerDiameter = j2Diameter - 2 * thickness - 2 * parameters['burshedServoAxis']['innerAxisMargin']

    j2ScrewHeadDiameter = parameters['burshedServo']['screwHeadDiameter']

    j3AxisWidthOffset = parameters['j3']['j3AxisWidthOffset']

    result = createBrushedServoBaseAxis(
            diameter=j2Diameter,
            innerDiameter=innerDiameter,
            width=j2KeyWidth,
            countersunkDepth=countersunkDepth,
            keyDepth=keyDepth,
            keyRadia=keyRadia,
            servoAxisOffset=j2ServoAxisOffset,
            enableBridging=True
        )

    diameter = parameters['microServoAxis']['diameter']
    thickness = parameters['microServoAxis']['thickness']
    keyWidth = parameters['microServoAxis']['clip']['width'] / 2
    countersunkDepth = parameters['microServoAxis']['clip']['countersunkDepth']
    clipCutoutMargin = parameters['microServoAxis']['clip']['cutoutMargin']
    keyDepth = parameters['microServoAxis']['clip']['keyDepth']
    keyRadia = parameters['microServoAxis']['clip']['keyRadia']

    axisMargin = parameters['microServoAxis']['axisMargin']

    servoTopCutout = parameters['microServo']['topCutout']
    servoBottomCutout = parameters['microServo']['bottomCutout']
    servoCutoutDepth = parameters['microServo']['cutoutDepth']
    servoTopMountHole = parameters['microServo']['topMountHole']
    servoBottomMountHole = parameters['microServo']['bottomMountHole']
    servoMountHoleDiameter = parameters['microServo']['mountHoleDiameter']

    servoCutoutWidth = parameters['microServo']['cutoutWidth']
    servoMountWidth = servoCutoutWidth + 2 * parameters['microServo']['bottomMountThickness']
    servoMountDepth = parameters['microServo']['mountDepth']
    servoMountHeight = parameters['microServo']['mountHeight']

    servoAxisOffset = parameters['j4']['j3AxisHeightOffset'] - j2ScrewHeadDiameter / 2

    j2Origin = (diameter / 2 + parameters['burshedServoAxis']['clip']['width'],
            servoAxisOffset + j2ScrewHeadDiameter / 2, -j2Diameter / 2 + diameter / 2)

    result = (
            result.rotate((0, 0, 0), (1, 0, 0), -90)
            .rotate((0, 0, 0), (0, 0, 1), -180)
            .translate(j2Origin)
        )

    result.faces('+X').faces('not >X').faces('>X').tag('toAxisPlane')
    result.faces('+Y').faces('>Y').tag('rotationAxisPlane')
    result.faces('+Z').faces('>Z').tag('rotationOrthAxisPlane')
    result.faces(tag='toAxisPlane').translate((j2ServoAxisOffset + j2KeyWidth, 0, 0)).tag('servoJ3AxisPlane')

    result = (
            result.faces('<X').workplane(origin=j2Origin)
            .moveTo(0, j2Diameter / 2)
            .radiusArc((0, -j2Diameter / 2), j2Diameter / 2)
            .close()
            .extrude(diameter)
            .moveTo(0, j2Diameter / 2)
            .lineTo(j2ScrewHeadDiameter / 2, j2Diameter / 2)
            .lineTo(j2ScrewHeadDiameter / 2, -j2Diameter / 2)
            .lineTo(-j2ScrewHeadDiameter / 2, -j2Diameter / 2)
            .close()
            .cutBlind(diameter)
        )

    radius = diameter / 2

    tempWorkplane = (
            result.faces('+Y').faces('not >Y').faces('>Y')
            .workplane(origin=(0,0,0))
        )

    result = (
            result.copyWorkplane(tempWorkplane)
            .moveTo(-radius, radius)
            .lineTo(0, radius)
            .lineTo(0, -radius)
            .lineTo(-radius, -radius)
            .close()
            .extrude(-servoAxisOffset)
            .moveTo(0, 0)
            .circle(radius)
            .cutThruAll()
            .copyWorkplane(tempWorkplane)
            .moveTo(-servoMountWidth / 2, 0)
            .lineTo(servoMountWidth / 2, 0)
            .lineTo(servoMountWidth / 2, -servoMountHeight)
            .lineTo(-servoMountWidth / 2, -servoMountHeight)
            .close()
            .cutBlind(-servoMountDepth)
        )

    result = (
            result.faces('-X')
            .faces('not <X').faces('not <X').faces('<X')
            .workplane(origin=(0,0,0))
            .moveTo(-j2Origin[1] + j2ScrewHeadDiameter / 2, -servoMountHeight)
            .lineTo(-j2Origin[1] - j2Diameter / 2, -servoMountHeight)
            .lineTo(-j2Origin[1] - j2Diameter / 2, j2Origin[2] - j2Diameter / 2)
            .lineTo(-j2Origin[1] + j2ScrewHeadDiameter / 2, j2Origin[2] - j2Diameter / 2)
            .close()
            .extrude(diameter)
            .moveTo(-j2Origin[1] + j2Diameter, j2Origin[2])
            .lineTo(-j2Origin[1] + j2Diameter / 2, j2Origin[2])
            .radiusArc((-j2Origin[1] - j2Diameter / 2, j2Origin[2]), j2Diameter / 2)
            .lineTo(-j2Origin[1] - j2Diameter, j2Origin[2])
            .lineTo(-j2Origin[1] - j2Diameter, j2Origin[2] - j2Diameter)
            .lineTo(-j2Origin[1] + j2Diameter, j2Origin[2] - j2Diameter)
            .close()
            .cutThruAll()
        )

    result2 = createMicroServoHolderAxis(
            diameter=diameter,
            thickness=thickness,
            width=keyWidth,
            countersunkDepth=countersunkDepth,
            keyDepth=keyDepth,
            keyRadia=keyRadia,
            servoAxisOffset=servoAxisOffset,
            axisMargin=axisMargin)

    result2 = (
            result2.rotate((0, 0, 0), (0, 0, 1), -90)
        )

    result2.faces('<Y').tag('fromAxisPlane')

    temp = keyDepth + countersunkDepth
    result2 = (
            result2.faces('<Y').workplane(origin=(0, 0, 0))
            .moveTo(-radius + temp / 2, 0)
            .rect(temp, 2 * radius).cutBlind(-keyWidth / 2)
        )

    result = result.union(result2)

    result = (
            result.copyWorkplane(tempWorkplane)
            .moveTo(0, radius)
            .radiusArc((radius, 0), radius)
            .lineTo(radius, radius)
            .close()
            .cutThruAll()
        )

    angle = parameters['j3']['roundingAngle'] / 180 * math.pi
    vertex0 = (radius, 0)
    vertex1 = (radius * (math.cos(angle)), -radius * math.sin(angle))
    vertex2 = (servoMountWidth / 2, -j2Diameter / 2 + radius)
    vertex3 = (-radius, -j2Diameter + radius - 0.1)

    result = (
            result.copyWorkplane(tempWorkplane)
            .moveTo(*vertex0)
            .threePointArc((vertex0[0] - 0.00001, vertex0[1] - 0.1), vertex1)
            .threePointArc((vertex1[0] - 0.1 * math.sin(angle), vertex1[1] - 0.1 * math.cos(angle)), vertex2)
            .tangentArcPoint((vertex3[0] - vertex2[0], vertex3[1] - vertex2[1]))
            .lineTo(vertex0[0], vertex3[1])
            .close()
            .cutThruAll()
        )

    result = (
            result.faces('+Z').faces('<Z')
        )

    vertex0 = projectToPlane(result.vertices('<Y').vertices('<X'), result.workplane())
    vertex1 = projectToPlane(result.vertices('>Y').vertices('>X'), result.workplane())
    vertex1 = vertex1 - (0.1, 0)
    vertex2 = (vertex0[0], vertex1[1])

    result = (
            result.workplane()
            .moveTo(*vertex0)
            .threePointArc(vertex1, (vertex1[0] - 0.0001, vertex1[1] + 0.1))
            .lineTo(*vertex2)
            .close()
            .cutThruAll()
        )

    vx0 = np.array((radius, 0))
    revolveAxis = np.array((0, 0, 0))
    vx1 = np.array((revolveAxis[0] + radius * math.sqrt(2), radius * math.sqrt(2)))

    cutoutSolid = (
            cq.Workplane('XY')
            .moveTo(*vx0)
            .lineTo(*vx1)
            .lineTo(vx1[0], vx0[1])
            .close()
            .revolve(360, tuple(revolveAxis), tuple(revolveAxis + (0, 1, 0)))
        )

    result = result.cut(cutoutSolid)

    return result

runCreateJ4 = False
def createJ4(asOneShape=True):
    width = parameters['j4']['width']

    diameter = parameters['microServoAxis']['diameter']
    thickness = parameters['microServoAxis']['thickness']
    keyWidth = parameters['microServoAxis']['clip']['width'] / 2
    clipHeight = parameters['microServoAxis']['clip']['height']
    countersunkDepth = parameters['microServoAxis']['clip']['countersunkDepth']
    keyDepth = parameters['microServoAxis']['clip']['keyDepth']
    keyRadia = parameters['microServoAxis']['clip']['keyRadia']

    innerDiameter = diameter - 2 * thickness - 2 * parameters['j4']['innerAxisMargin']

    clampRadius = parameters['microServoAxis']['clampRadius']
    clampDepth = parameters['microServoAxis']['clampDepth']
    
    screwHeadDiameter = parameters['microServoAxis']['screwHeadDiameter']
    axisDepthOffset = parameters['microServoAxis']['servoAxisOffset']

    screwHoleDiameter = parameters['microServoAxis']['screwHoleDiameter']
    servoMountDepth = parameters['microServo']['mountDepth']

    j5ServoAxisOffset = parameters['microServoAxis']['servoAxisOffset']
    j4ServoAxisWidthOffset = parameters['j3']['j3AxisWidthOffset']

    j4ServoAxisHeightOffset = parameters['j4']['j3AxisHeightOffset'] - keyWidth
    j4AxisLenght = j4ServoAxisHeightOffset - servoMountDepth + 2 * keyWidth

    j4AxisInnerDiameter = parameters['j4']['innerDiameter']

    clipCutoutMargin = parameters['j4']['cutoutMargin']

    length = parameters['j4']['j2Offset'] - servoMountDepth - parameters['burshedServo']['screwHeadDiameter'] / 2

    radius = diameter / 2

    result = (
            createMicroServoBaseAxis(
                diameter=diameter,
                innerDiameter=innerDiameter,
                screwHoleDiameter=screwHoleDiameter,
                width=keyWidth,
                countersunkDepth=countersunkDepth,
                keyDepth=keyDepth,
                keyRadia=keyRadia,
                servoAxisOffset=j5ServoAxisOffset,
                clampRadius=clampRadius,
                clampDepth=clampDepth,
                screwHeadDiameter=screwHeadDiameter)
            .translate((keyWidth, 0, 0))
        )

    result.faces('-X').faces(cq.NearestToPointSelector((keyWidth, 0, 0))).tag('fromAxisPlane')
    result.faces('-X').faces('<X').tag('servoJ5AxisPlane')


    result = (
            result.faces('>X').workplane()
            .circle(radius)
            .circle(screwHeadDiameter / 2)
            .extrude(axisDepthOffset - keyWidth)
            .faces('>X').workplane()
            .moveTo(0, diameter / 2)
            .radiusArc((0, -radius), radius)
            .lineTo(length - j4AxisLenght, -radius)
            .lineTo(length - j4AxisLenght, radius)
            .close()
            .extrude(-width)
        )

    result = (
            result.faces('<X').workplane()
            .moveTo(0, 0)
            .circle(radius + clipCutoutMargin + clipHeight - countersunkDepth)
            .circle(radius)
            .cutBlind(-width + axisDepthOffset - keyWidth)
            .faces(cq.NearestToPointSelector(((-width + axisDepthOffset - keyWidth) / 2, radius , 0)))
            .edges('not |X').fillet(clipCutoutMargin * 0.5)
        )

    tempLength = parameters['j4']['taperDist']

    result2 = (
            createMicroServoBaseAxis(
                diameter=diameter,
                innerDiameter=innerDiameter,
                screwHoleDiameter=screwHoleDiameter,
                width=keyWidth,
                countersunkDepth=countersunkDepth,
                keyDepth=keyDepth,
                keyRadia=keyRadia,
                servoAxisOffset=j4ServoAxisHeightOffset,
                clampRadius=clampRadius,
                clampDepth=clampDepth,
                screwHeadDiameter=screwHeadDiameter)
            .faces('>X').workplane()
            .moveTo(j4ServoAxisWidthOffset, radius)
            .lineTo(0, radius)
            .radiusArc((0, -radius), -radius)
            .lineTo(j4ServoAxisWidthOffset, -radius)
            .close()
            .extrude(tempLength)
            .faces('>X').workplane()
            .moveTo(0, 0)
            .circle(j4AxisInnerDiameter / 2)
            .cutBlind(-tempLength - j4AxisLenght + servoMountDepth - 2 * keyWidth
                    + clampDepth + 2)
        )

    translateVec = (-j4ServoAxisWidthOffset, length - j4AxisLenght + keyWidth, 0)
    result2 = (
            result2.rotate((0, 0, 0), (0, 0, 1), -90)
            .translate(translateVec)
        )


    result2.faces('+Y').faces(cq.NearestToPointSelector(translateVec)).tag('toAxisPlane')
    result2.faces('+Y').faces('>Y').tag('servoJ4AxisPlane')

    result = result.union(result2)

    result = (
            result.faces('>Z').workplane(origin=(0,0,0))
            .moveTo(-j4ServoAxisWidthOffset + radius, length - j4AxisLenght)
            .lineTo(axisDepthOffset + keyWidth, length - j4AxisLenght)
            .lineTo(axisDepthOffset + keyWidth, length - j4AxisLenght - 45)
            .close()
            .moveTo(-j4ServoAxisWidthOffset - radius, length - j4AxisLenght - 10)
            .lineTo(-j4ServoAxisWidthOffset - radius, length - j4AxisLenght - tempLength)
            .lineTo(axisDepthOffset + keyWidth - width, length - j4AxisLenght - tempLength)
            .close()
            .cutThruAll()
            .faces('>X').edges('|Z').fillet(diameter)
            .faces('- X').faces('<X').edges('|Z').fillet(diameter)
        )

    result.faces('+X').faces('>X').tag('rotationAxisPlane')
    result.faces('+Z').faces('>Z').tag('rotationOrthAxisPlane')

    vx0 = np.array((-j4ServoAxisWidthOffset + radius, length - j4AxisLenght))
    revolveAxis = np.array((-j4ServoAxisWidthOffset, 0, 0))
    vx1 = np.array((revolveAxis[0] + 45, length - j4AxisLenght - 45))

    cutoutSolid = (
            cq.Workplane('XY')
            .moveTo(*vx0)
            .lineTo(*vx1)
            .lineTo(vx1[0], vx0[1])
            .close()
            .revolve(360, tuple(revolveAxis), tuple(revolveAxis + (0, 1, 0)))
        )

    result = result.cut(cutoutSolid)

    topBottomKeyOffset = parameters['j4']['topBottomKey']['distFromJ4']
    topBottomKeyHeight = parameters['j4']['topBottomKey']['height']
    topBottomKeyTaper = parameters['j4']['topBottomKey']['taperDist']
    topBottomKeyMargin = parameters['j4']['topBottomKey']['margin']

    innerRadius = j4AxisInnerDiameter / 2
    cutoutSolid = (
            cq.Workplane('YZ').workplane(offset=-j4ServoAxisWidthOffset)
            .moveTo(vx0[1] - topBottomKeyOffset, radius)
            .lineTo(vx0[1] - topBottomKeyOffset, innerRadius)
            .lineTo(vx0[1] - topBottomKeyHeight, innerRadius + topBottomKeyTaper)
            .lineTo(vx0[1] - topBottomKeyHeight, -innerRadius - topBottomKeyTaper)
            .lineTo(vx0[1] - topBottomKeyOffset, -innerRadius)
            .lineTo(vx0[1] - topBottomKeyOffset, -radius)
            .lineTo(vx0[1] - length, -radius)
            .lineTo(vx0[1] - length, radius)
            .close()
            .extrude(-radius)
            .moveTo(vx0[1] - topBottomKeyOffset, radius)
            .lineTo(vx0[1] - topBottomKeyOffset, -radius)
            .lineTo(vx0[1] - length, -radius)
            .lineTo(vx0[1] - length, radius)
            .close()
            .extrude(j4ServoAxisWidthOffset + j5ServoAxisOffset + keyWidth)
        )

    global temp
    cutoutSolidMargin = cutoutSolid.union(cutoutSolid.translate((0,0,0)).shell(topBottomKeyMargin))

    resultBottom = result.cut(cutoutSolidMargin)
    resultTop = result.intersect(cutoutSolid)

    if asOneShape:
        return resultTop.union(resultBottom)

    return resultTop, resultBottom

runCreateJ5 = False
def createJ5():
    diameter = parameters['microServoAxis']['diameter']
    thickness = parameters['microServoAxis']['thickness']
    keyWidth = parameters['microServoAxis']['clip']['width'] / 2
    clipHeight = parameters['microServoAxis']['clip']['height']
    countersunkDepth = parameters['microServoAxis']['clip']['countersunkDepth']
    clipCutoutMargin = parameters['microServoAxis']['clip']['cutoutMargin']
    keyDepth = parameters['microServoAxis']['clip']['keyDepth']
    keyRadia = parameters['microServoAxis']['clip']['keyRadia']

    servoAxisOffset = parameters['microServoAxis']['servoAxisOffset']

    axisMargin = parameters['microServoAxis']['axisMargin']

    servoCutoutWidth = parameters['microServo']['cutoutWidth']
    servoCutoutTop = parameters['j5']['servoTopCutout']

    j3AxisWidthOffset = parameters['j3']['j3AxisWidthOffset']

    j6AxisLengthOffset = parameters['j6']['lengthOffset']

    radius = diameter / 2

    result = createMicroServoHolderAxis(
            diameter=diameter,
            thickness=thickness,
            width=keyWidth,
            countersunkDepth=countersunkDepth,
            keyDepth=keyDepth,
            keyRadia=keyRadia,
            servoAxisOffset=servoAxisOffset,
            axisMargin=axisMargin)

    result.faces('+X').faces('>X').tag('toAxisPlane')
    result.faces('+Y').faces('>Y').tag('rotationAxisPlane')
    result.faces('+Z').faces('>Z').tag('rotationOrthAxisPlane')

    tempLength = j3AxisWidthOffset - servoAxisOffset

    result = (
            result.faces('<X').workplane()
            .circle(radius)
            .extrude(tempLength)
        )

    result = (
            result.faces('<X').workplane()
            .moveTo(-radius, servoCutoutTop)
            .lineTo(servoCutoutWidth / 2, servoCutoutTop)
            .lineTo(servoCutoutWidth / 2, -radius)
            .lineTo(-radius, -radius)
            .close()
            .cutBlind(-tempLength)
        )

    result = (
            result.faces('<X').workplane()
            .moveTo(-radius, servoCutoutWidth / 2)
            .lineTo(radius, servoCutoutWidth / 2)
            .lineTo(radius, -servoCutoutWidth / 2)
            .lineTo(-radius, -servoCutoutWidth / 2)
            .close()
            .cutBlind(-servoCutoutTop)
        )

    tempFace = result.faces('<X')

    result2 = createMicroServoHolderAxis(
            diameter=diameter,
            thickness=thickness,
            width=keyWidth,
            countersunkDepth=countersunkDepth,
            keyDepth=keyDepth,
            keyRadia=keyRadia,
            servoAxisOffset=servoAxisOffset,
            axisMargin=axisMargin)

    tempLength2 = j6AxisLengthOffset - servoAxisOffset

    result2 = (
            result2.faces('<X').workplane()
            .circle(radius)
            .extrude(tempLength2)
        )

    result2 = (
            result2.faces('<X').workplane()
            .moveTo(-servoCutoutWidth / 2, servoCutoutTop)
            .lineTo(servoCutoutWidth / 2, servoCutoutTop)
            .lineTo(servoCutoutWidth / 2, -radius)
            .lineTo(-servoCutoutWidth / 2, -radius)
            .close()
            .cutBlind(-tempLength2)
        )

    result2 = (
            result2.faces('<X').workplane()
            .moveTo(-servoCutoutTop, radius)
            .lineTo(servoCutoutTop, radius)
            .lineTo(servoCutoutTop, -radius)
            .lineTo(-servoCutoutTop, -radius)
            .close()
            .cutBlind(-servoCutoutWidth / 2)
        )

    result2 = (
            result2.rotate((0, 0, 0), (1, 0, 0), -90)
            .rotate((0, 0, 0), (0, 0, 1), -90)
            .translate((-j3AxisWidthOffset, -j6AxisLengthOffset, 0))
        )

    result2.faces('<Y').tag('fromAxisPlane')

    temp = keyDepth + countersunkDepth
    result2 = (
            result2.faces('<Y').workplane(origin=(-j3AxisWidthOffset,0,0))
            .moveTo(radius - temp / 2, 0)
            .rect(temp, 2 * radius).cutBlind(-keyWidth / 2)
        )

    result = result.union(result2)

    result = (
            result.copyWorkplane(tempFace).workplane(offset=radius)
            .moveTo(-radius, servoCutoutTop)
            .lineTo(servoCutoutWidth / 2, servoCutoutTop)
            .lineTo(servoCutoutWidth / 2, -radius)
            .lineTo(-radius, -radius)
            .close()
            .cutBlind(-tempLength - radius)
        )

    xyWorkPlane = cq.Workplane('XY')

    temp = result.faces('-Z').faces('>Z').edges('>Y').vertices('>X')
    vertex0 = projectToPlane(temp, xyWorkPlane)

    temp = result.faces('+Y').faces('not >Y').faces('>Y').vertices('>Z')
    vertex1 = projectToPlane(temp, xyWorkPlane)

    temp = result.faces('+Y').faces('not <Y').faces('<Y').edges('>Z').vertices('>X')
    vertex2 = projectToPlane(temp, xyWorkPlane)

    result = (
            result.copyWorkplane(xyWorkPlane)
            .moveTo(vertex0[0], vertex0[1])
            .lineTo(*vertex1)
            .lineTo(*vertex2)
            .lineTo(vertex2[0] - 100, vertex2[1])
            .lineTo(vertex2[0] - 100, vertex0[1])
            .close()
            .cutThruAll()
        )

    return result

runCreateJ6 = False
def createJ6():
    diameter = parameters['microServoAxis']['diameter']
    thickness = parameters['microServoAxis']['thickness']
    keyWidth = parameters['microServoAxis']['clip']['width'] / 2
    countersunkDepth = parameters['microServoAxis']['clip']['countersunkDepth']
    keyDepth = parameters['microServoAxis']['clip']['keyDepth']
    keyRadia = parameters['microServoAxis']['clip']['keyRadia']

    innerDiameter = diameter - 2 * thickness - 2 * parameters['microServoAxis']['innerAxisMargin']

    clampRadius = parameters['microServoAxis']['clampRadius']
    clampDepth = parameters['microServoAxis']['clampDepth']
    
    screwHeadDiameter = parameters['microServoAxis']['screwHeadDiameter']

    screwHoleDiameter = parameters['microServoAxis']['screwHoleDiameter']

    servoAxisOffset = parameters['microServoAxis']['servoAxisOffset']

    radius = diameter / 2

    result = (
            createMicroServoBaseAxis(
                diameter=diameter,
                innerDiameter=innerDiameter,
                screwHoleDiameter=screwHoleDiameter,
                width=keyWidth,
                countersunkDepth=countersunkDepth,
                keyDepth=keyDepth,
                keyRadia=keyRadia,
                servoAxisOffset=servoAxisOffset,
                clampRadius=clampRadius,
                clampDepth=clampDepth,
                screwHeadDiameter=screwHeadDiameter)
            .translate((keyWidth, 0, 0))
        )

    result.faces('-X').faces(cq.NearestToPointSelector((keyWidth, 0, 0))).tag('toAxisPlane')
    result.faces('-X').faces('<X').tag('servoJ6AxisPlane')

    rectWidth = 12.2
    radius0 = 4.8
    radius1 = rectWidth / 2
    rectHeight = 10
    rectDepth = 1.0
    gripperClipDepth = 2.2
    baseOffsetRectWidth = rectWidth + 1.6
    baseOffsetRectDepth = 0.8

    angle = math.acos(radius0 / radius1)
    v0 = np.array((math.cos(-angle), math.sin(-angle)))

    temp = radius0
    p0 = temp * v0
    p1 = (0, temp)
    p2 = (-p0[0], p0[1])

    result = (
            result.faces('>X').workplane()
            .rect(baseOffsetRectWidth, baseOffsetRectWidth)
            .extrude(baseOffsetRectDepth)
        )

    baseWorkplane = result.faces('>X').workplane()

    result = (
            result.copyWorkplane(baseWorkplane)
            .lineTo(*p0)
            .threePointArc(p1, p2)
            .close()
            .extrude(gripperClipDepth)
        )

    r = radius1
    p0 = (r * v0)
    p1 = (0, -r)
    p2 = (-p0[0], p0[1])

    result = (
            result.copyWorkplane(baseWorkplane)
            .lineTo(*p0)
            .threePointArc(p1, p2)
            .close()
            .extrude(gripperClipDepth)
        )

    result = (
            result.faces('>X').workplane(origin=(0,0,0))
            .circle(radius1)
            .extrude(rectDepth)
        )

    p0 = (radius0, rectHeight / 2)
    p1 = (radius0, -radius1)
    p2 = (-radius0, -radius1)
    p3 = (-radius0, rectHeight / 2)

    result = (
            result.faces('>X').workplane(origin=(0,0,0))
            .moveTo(*p0)
            .lineTo(*p1)
            .lineTo(*p2)
            .lineTo(*p3)
            .close()
            .circle(2 * radius1)
            .cutBlind(-rectDepth)
        )

    result = (
            result.faces('>X').workplane(origin=(0,0,0))
            .rect(rectWidth, rectWidth)
            .extrude(rectDepth)
        )

    result = (
            result.faces('>X').workplane(origin=(0,0,0))
            .circle(screwHeadDiameter / 2)
            .cutBlind(-2 * rectDepth - gripperClipDepth - baseOffsetRectDepth)
        )

    result.faces('+Z').faces('>Z').tag('rotationAxisPlane')
    result.faces('+Y').faces('>Y').tag('rotationOrthAxisPlane')

    return result

runCreateMicroServoClip = False
def createMicroServoClip():
    diameter = parameters['microServoAxis']['diameter']
    width = parameters['microServoAxis']['clip']['width']
    widthCompression = parameters['microServoAxis']['clip']['widthCompression']
    height = parameters['microServoAxis']['clip']['height']
    countersunkDepth = parameters['microServoAxis']['clip']['countersunkDepth']
    keyDepth = parameters['microServoAxis']['clip']['keyDepth']
    keyRadia = parameters['microServoAxis']['clip']['keyRadia']
    edgeMargin = parameters['microServoAxis']['clip']['edgeMargin']
    clipOpeningWidth = parameters['microServoAxis']['clip']['clipOpeningWidth']

    result = createLockingClip(
            diameter = diameter,
            width=width - widthCompression,
            height=height,
            countersunkDepth=countersunkDepth,
            keyDepth=keyDepth,
            keyRadia=keyRadia,
            edgeMargin=edgeMargin,
            clipOpeningWidth=clipOpeningWidth)

    (
        result.faces('>X')
        .translate((-width / 2 + edgeMargin, 0, 0))
        .tag('axisPlane')
    )

    result = (
        result.faces('<X').workplane(origin=(0,0,0))
        .circle(diameter / 2 - countersunkDepth - keyDepth + edgeMargin)
        .cutBlind(-width)
    )

    return result

runCreateBrushedServoClip = False
def createBrushedServoClip():
    diameter = parameters['burshedServoAxis']['diameter']
    width = parameters['burshedServoAxis']['clip']['width']
    height = parameters['burshedServoAxis']['clip']['height']
    countersunkDepth = parameters['burshedServoAxis']['clip']['countersunkDepth']
    keyDepth = parameters['burshedServoAxis']['clip']['keyDepth']
    keyRadia = parameters['burshedServoAxis']['clip']['keyRadia']
    edgeMargin = parameters['burshedServoAxis']['clip']['edgeMargin']
    clipOpeningWidth = parameters['burshedServoAxis']['clip']['clipOpeningWidth']

    result = createLockingClip(
            diameter = diameter,
            width=width,
            height=height,
            countersunkDepth=countersunkDepth,
            keyDepth=keyDepth,
            keyRadia=keyRadia,
            edgeMargin=edgeMargin,
            clipOpeningWidth=clipOpeningWidth)

    (
        result.faces('>X')
        .translate((-width / 2 + edgeMargin, 0, 0))
        .tag('axisPlane')
    )

    result = (
        result.faces('<X').workplane(origin=(0,0,0))
        .circle(diameter / 2 - countersunkDepth - keyDepth + edgeMargin)
        .cutBlind(-width)
    )

    return result

runImportMicroServo = False
def importMicroServo():
    result = cq.importers.importStep('microServo.step')
    (
        result
        .faces('+Z')
        .faces('>Z').translate((0,0,-4.45)).tag('axisPlane')
    )

    (
        result
        .faces('>Y').tag('rotationAxisPlane')
    )

    return result

runImportBrushedServo = False
def importBrushedServo():
    result = cq.importers.importStep('brushedServo.step')
    (
        result
        .faces('-Z')
        .faces('<Z').translate((0,0,-2.4)).tag('axisPlane')
    )

    (
        result
        .faces('<X').tag('rotationAxisPlane')
    )

    return result

runImportBrushedServoHorn = False
def importBrushedServoHorn():
    result = cq.importers.importStep('brushedServoHorn.step')

    temp = result.faces('+Z').faces('>Z')
    cOfMass = temp.val().Center().toTuple()
    (
        temp.translate((-cOfMass[0], -cOfMass[1], -cOfMass[2])).tag('axisPlane')
    )

    (
        result
        .faces('<X').tag('rotationAxisPlane')
    )

    return result

runCreateMicroPcbHolder = False
def createMicroPcbHolder():
    screwHoleOffset = 4.7 / 2
    screwHoleDiameter = 2.4
    screwHoleDepth = 2
    pcbWidth = 20
    pcbThickness = 1.6 + 0.2
    pcbMountOffset = 6
    pcbMountThickness = 1.2
    pcbMountLength = 8

    temp = screwHoleOffset + pcbMountOffset + pcbThickness / 2 + pcbMountThickness
    result = (
            cq.Workplane('XY')
            .circle(screwHoleDiameter / 2)
            .moveTo(0, -screwHoleOffset + temp / 2)
            .rect(pcbWidth + 2 * pcbMountThickness,
                    temp)
            .extrude(pcbMountLength)
        )

    result = (
            result.moveTo(0, pcbMountOffset)
            .rect(pcbWidth, pcbThickness)
            .cutThruAll()
        )

    temp = pcbMountOffset + pcbThickness / 2 + pcbMountThickness - screwHoleOffset
    result = (
            result.moveTo(0, screwHoleOffset + temp / 2)
            .rect(pcbWidth - 2 * pcbMountThickness, temp)
            .cutThruAll()
            
        )

    temp = pcbMountOffset + pcbThickness / 2 + pcbMountThickness + screwHoleOffset
    result = (
            result.faces('>Z').workplane(origin=(0,0,0))
            .moveTo(0, -screwHoleOffset + temp / 2)
            .rect(pcbWidth - 2 * pcbMountThickness, temp)
            .cutBlind(-pcbMountLength + screwHoleDepth)
            
        )

    result = (
            result.faces('+Z').faces('<Z').edges('|Y').fillet(screwHoleDepth)
        )
    return result

##############################################################################

def setParameters():
    global parameters

    parameters['bridgingHeight'] = 0.3

    parameters['microServo'] = dict()
    parameters['microServo']['topCutout'] = 6.4
    parameters['microServo']['bottomCutout'] = 17
    parameters['microServo']['bottomMountThickness'] = 1.5
    parameters['microServo']['cutoutWidth'] = 13
    parameters['microServo']['cutoutDepth'] = 5.5
    parameters['microServo']['mountHeight'] = 22
    parameters['microServo']['topMountHole'] = 8.9
    parameters['microServo']['bottomMountHole'] = 19.5
    parameters['microServo']['mountHoleDiameter'] = 1.6
    parameters['microServo']['mountHoleFillet'] = 0.5
    parameters['microServo']['mountDepth'] = 7.2
    parameters['microServo']['screwWallThickness'] = 1.0

    parameters['microServoAxis'] = dict()
    parameters['microServoAxis']['diameter'] = 30
    parameters['microServoAxis']['thickness'] = 3
    parameters['microServoAxis']['innerAxisMargin'] = 0.8
    parameters['microServoAxis']['clip'] = dict()
    parameters['microServoAxis']['clip']['height'] = 2.0
    parameters['microServoAxis']['clip']['width'] = 6.0
    parameters['microServoAxis']['clip']['widthCompression'] = 0.2
    parameters['microServoAxis']['clip']['countersunkDepth'] = 0.8
    parameters['microServoAxis']['clip']['cutoutMargin'] = 2.4
    parameters['microServoAxis']['clip']['keyDepth'] = 0.8
    parameters['microServoAxis']['clip']['keyRadia'] = 0.3
    parameters['microServoAxis']['clip']['edgeMargin'] = 0.2
    parameters['microServoAxis']['clip']['clipOpeningWidth'] = 0.8
    parameters['microServoAxis']['clamp'] = dict()
    parameters['microServoAxis']['clamp']['radius'] = 9.8
    parameters['microServoAxis']['clamp']['height'] = 5.3
    parameters['microServoAxis']['clamp']['innerDiameter'] = 4.0
    parameters['microServoAxis']['clamp']['innerDiameterTaper'] = 4
    parameters['microServoAxis']['clamp']['baseThickness'] = 0.6
    parameters['microServoAxis']['clamp']['screwHoleDiameter'] = 2.8
    parameters['microServoAxis']['clamp']['openingWidth'] = 1.0
    parameters['microServoAxis']['clamp']['rounding'] = 1.6
    parameters['microServoAxis']['clamp']['innerFilletRadius'] = 0.6
    parameters['microServoAxis']['clamp']['shellThickness'] = 1.2
    parameters['microServoAxis']['clampRadius'] = 10
    parameters['microServoAxis']['clampDepth'] = 5.5
    parameters['microServoAxis']['screwHoleDiameter'] = 2.8
    parameters['microServoAxis']['screwHeadDiameter'] = 4.4
    parameters['microServoAxis']['servoAxisOffset'] = 10
    parameters['microServoAxis']['axisMargin'] = 1

    parameters['burshedServo'] = dict()
    parameters['burshedServo']['topCutout'] = 11
    parameters['burshedServo']['bottomCutout'] = 31
    parameters['burshedServo']['bottomMountThickness'] = 4
    parameters['burshedServo']['cutoutWidth'] = 21
    parameters['burshedServo']['cutoutDepth'] = 10.5
    parameters['burshedServo']['mountHeight'] = 40
    parameters['burshedServo']['mountDepth'] = 17.3
    parameters['burshedServo']['topMountHole'] = 14.8
    parameters['burshedServo']['bottomMountHole'] = 34.7
    parameters['burshedServo']['mountHoleWidth'] = 9.8
    parameters['burshedServo']['mountHoleDiameter'] = 2.9
    parameters['burshedServo']['mountHoleDepth'] = 6
    parameters['burshedServo']['tabWidth'] = 2
    parameters['burshedServo']['tabDepth'] = 3
    parameters['burshedServo']['screwWallThickness'] = 2.0
    parameters['burshedServo']['screwHoleDiameter'] = 3.2
    parameters['burshedServo']['screwHeadDiameter'] = 6

    parameters['burshedServoAxis'] = dict()
    parameters['burshedServoAxis']['diameter'] = 65
    parameters['burshedServoAxis']['thickness'] = 4
    parameters['burshedServoAxis']['innerAxisMargin'] = 0.5
    parameters['burshedServoAxis']['servoAxisOffset'] = 4
    parameters['burshedServoAxis']['mountThickness'] = 14
    parameters['burshedServoAxis']['secScrewOffset'] = 20
    parameters['burshedServoAxis']['clip'] = dict()
    parameters['burshedServoAxis']['clip']['height'] = 2.0
    parameters['burshedServoAxis']['clip']['width'] = 8.0
    parameters['burshedServoAxis']['clip']['countersunkDepth'] = 0.0
    parameters['burshedServoAxis']['clip']['cutoutMargin'] = 4
    parameters['burshedServoAxis']['clip']['keyDepth'] = 1.6
    parameters['burshedServoAxis']['clip']['keyRadia'] = 0.6
    parameters['burshedServoAxis']['clip']['edgeMargin'] = 0.2
    parameters['burshedServoAxis']['clip']['clipOpeningWidth'] = 1.6

    parameters['j1'] = dict()
    parameters['j1']['j2AxisXOffset'] = 28
    parameters['j1']['j2AxisYOffset'] = 16
    parameters['j1']['j2BaseAxisExtraThinckness'] = 8
    parameters['j1']['servoCutoutHight'] = 19
    parameters['j1']['roundingRadius'] = 10
    parameters['j1']['roundingAngle'] = 20

    parameters['j2'] = dict()
    parameters['j2']['length'] = 180
    parameters['j2']['servo2Angle'] = 10
    parameters['j2']['servo3Angle'] = 30
    parameters['j2']['servo2SideWidth'] = 50
    parameters['j2']['servo3SideWidth'] = 45
    parameters['j2']['servo3SideOffset'] = 16
    parameters['j2']['servoCutoutTop'] = 39
    parameters['j2']['servoCutoutBottom'] = 20
    parameters['j2']['servoCutoutWidth'] = 22
    parameters['j2']['roundingHeight'] = 16
    parameters['j2']['roundingStartOffset'] = 5
    parameters['j2']['roundingRadius'] = 150

    parameters['j2']['topBottomKey'] = dict()
    parameters['j2']['topBottomKey']['length'] = 55.0
    parameters['j2']['topBottomKey']['thickness'] = 6.0
    parameters['j2']['topBottomKey']['margin'] = 0.3

    parameters['j3'] = dict()
    parameters['j3']['j3AxisWidthOffset'] = 21
    parameters['j3']['roundingAngle'] = 44

    parameters['j4'] = dict()
    parameters['j4']['j2Offset'] = 180
    parameters['j4']['width'] = 20
    parameters['j4']['j3AxisHeightOffset'] = 60
    parameters['j4']['taperDist'] = 80
    parameters['j4']['innerDiameter'] = 18
    parameters['j4']['cutoutMargin'] = 8
    parameters['j4']['innerAxisMargin'] = 0.8

    parameters['j4']['topBottomKey'] = dict()
    parameters['j4']['topBottomKey']['distFromJ4'] = 20
    parameters['j4']['topBottomKey']['height'] = 10
    parameters['j4']['topBottomKey']['taperDist'] = 1
    parameters['j4']['topBottomKey']['margin'] = 0.3

    parameters['j5'] = dict()
    parameters['j5']['servoTopCutout'] = 11.5

    parameters['j6'] = dict()
    parameters['j6']['lengthOffset'] = 32

setParameters()

if runCreateJ0:
    runCreateRobot = False
    j0 = createJ0()

if runCreateJ1:
    runCreateRobot = False
    j1 = createJ1()

if runCreateJ2:
    runCreateRobot = False
    j2Top, j2Bottom = createJ2(False)

if runCreateJ3:
    runCreateRobot = False
    j3 = createJ3()

if runCreateJ4:
    runCreateRobot = False
    j4Top, j4Bottom = createJ4(False)

if runCreateJ5:
    runCreateRobot = False
    j5 = createJ5()

if runCreateJ6:
    runCreateRobot = False
    j6 = createJ6()

if runImportMicroServo:
    runCreateRobot = False
    microServo = importMicroServo()

if runImportBrushedServo:
    runCreateRobot = False
    brushedServo = importBrushedServo()

if runImportBrushedServoHorn:
    runCreateRobot = False
    brushedServoHorn = importBrushedServoHorn()

if runCreateMicroServoClip:
    runCreateRobot = False
    microServoClip = createMicroServoClip()

if runCreateBrushedServoClip:
    runCreateRobot = False
    brushedServoClip = createBrushedServoClip()

if runCreateMicroServoAxisClamp:
    runCreateRobot = False
    microServoAxisClamp = createMicroServoAxisClamp()

if True:
    runCreateRobot = False

    j0 = createJ0().translate((0, 0, 0))
    j1 = createJ1().translate((160, 0, 0))
    j2Tuple = createJ2(False)
    j2Top = j2Tuple[0].translate((240, 0, 0))
    j2Bottom = j2Tuple[1].translate((240, 0, 0))
    j3 = createJ3().translate((320, 0, 0))
    j4Tuple = createJ4(False)
    j4Top = j4Tuple[0].translate((400, 0, 0))
    j4Bottom = j4Tuple[1].translate((400, 0, 0))
    j5 = createJ5().translate((480, 0, 0))
    j6 = createJ6().translate((520, 0, 0))

    microServoClip = createMicroServoClip().translate((560, 0, 0))
    microServoAxisClamp = createMicroServoAxisClamp().translate((600, 0, 0))
    brushedServoClip = createBrushedServoClip().translate((640, 0, 0))

if runCreateMicroPcbHolder:
    runCreateRobot = False
    microPcbHolder = createMicroPcbHolder()

def createRobot():
    c = 0.9
    a = 0.3
    colors = [
            cq.Color(c, 0.4 * c, 0.0, a),
            cq.Color(c, 0.0 * c, 0.0, a)
        ]

    robotAsm = (
            cq.Assembly()
            .add(createJ0(), name='j0', color=colors[0])
            .add(createJ1(), name='j1', color=colors[1])
            .add(createJ2(), name='j2', color=colors[0])
            .add(createJ3(), name='j3', color=colors[1])
            .add(createJ4(), name='j4', color=colors[0])
            .add(createJ5(), name='j5', color=colors[1])
            .add(createJ6(), name='j6', color=colors[0])
        )

    j1Angle = 0
    j2Angle = 110
    j3Angle = 130
    j4Angle = 0
    j5Angle = 20
    j6Angle = 0

    j2Servo2Angle = parameters['j2']['servo2Angle']

    j2Angle += j2Servo2Angle
    j3Angle += j2Servo2Angle

    (
        robotAsm
        .constrain('j0', 'Fixed')
        .constrain('j0?fromAxisPlane', 'j1?toAxisPlane', 'Plane')
        .constrain('j1?fromAxisPlane', 'j2?toAxisPlane', 'Plane')
        .constrain('j2?fromAxisPlane', 'j3?toAxisPlane', 'Plane')
        .constrain('j3?fromAxisPlane', 'j4?toAxisPlane', 'Plane')
        .constrain('j4?fromAxisPlane', 'j5?toAxisPlane', 'Plane')
        .constrain('j5?fromAxisPlane', 'j6?toAxisPlane', 'Plane')
    )
    (
        robotAsm
        .constrain('j0?rotationAxisPlane', 'j1?rotationAxisPlane', 'Axis', param=j1Angle)
        .constrain('j0?rotationAxisPlane', 'j1?rotationOrthAxisPlane', 'Axis', param=j1Angle + 90)
        .constrain('j1?rotationAxisPlane', 'j2?rotationAxisPlane', 'Axis', param=j2Angle)
        .constrain('j1?rotationAxisPlane', 'j2?rotationOrthAxisPlane', 'Axis', param=j2Angle + 90)
        .constrain('j2?rotationAxisPlane', 'j3?rotationAxisPlane', 'Axis', param=j3Angle)
        .constrain('j2?rotationAxisPlane', 'j3?rotationOrthAxisPlane', 'Axis', param=j3Angle + 90)
        .constrain('j3?rotationOrthAxisPlane', 'j4?rotationAxisPlane', 'Axis', param=j4Angle + 90)
        .constrain('j3?rotationOrthAxisPlane', 'j4?rotationOrthAxisPlane', 'Axis', param=j4Angle)
        .constrain('j4?rotationOrthAxisPlane', 'j5?rotationAxisPlane', 'Axis', param=j5Angle + 90)
        .constrain('j4?rotationOrthAxisPlane', 'j5?rotationOrthAxisPlane', 'Axis', param=j5Angle)
        .constrain('j5?rotationOrthAxisPlane', 'j6?rotationAxisPlane', 'Axis', param=j6Angle)
        .constrain('j5?rotationOrthAxisPlane', 'j6?rotationOrthAxisPlane', 'Axis', param=j6Angle + 90)
    )

    if True:
        temp0 = importBrushedServo()
        temp1 = importMicroServo()
        robotAsm = (
                robotAsm
                .add(temp0, name='servoJ1')
                .add(temp0.translate((0,0,0)), name='servoJ2')
                .add(temp0.translate((0,0,0)), name='servoJ3')
                .add(temp1, name='servoJ4')
                .add(temp1.translate((0,0,0)), name='servoJ5')
                .add(temp1.translate((0,0,0)), name='servoJ6')
            )

        (
            robotAsm
            .constrain('j0?servoJ1AxisPlane', 'servoJ1?axisPlane', 'Plane')
            .constrain('j1@faces@+Z', 'servoJ1?rotationAxisPlane', 'Axis')
            .constrain('j1?servoJ2AxisPlane', 'servoJ2?axisPlane', 'Plane')
            .constrain('j2@faces@-Z', 'servoJ2?rotationAxisPlane', 'Axis')
            .constrain('j3?servoJ3AxisPlane', 'servoJ3?axisPlane', 'Plane')
            .constrain('j2@faces@-Z', 'servoJ3?rotationAxisPlane', 'Axis', param=parameters['j2']['servo3Angle'])
            .constrain('j4?servoJ4AxisPlane', 'servoJ4?axisPlane', 'Plane')
            .constrain('j3@faces@+Z', 'servoJ4?rotationAxisPlane', 'Axis')
            .constrain('j4?servoJ5AxisPlane', 'servoJ5?axisPlane', 'Plane')
            .constrain('j5@faces@+Z', 'servoJ5?rotationAxisPlane', 'Axis')
            .constrain('j6?servoJ6AxisPlane', 'servoJ6?axisPlane', 'Plane')
            .constrain('j5@faces@+X', 'servoJ6?rotationAxisPlane', 'Axis')
        )

    if True:
        temp = importBrushedServoHorn()
        robotAsm = (
                robotAsm
                .add(temp, name='servoHornJ1')
                .add(temp.translate((0,0,0)), name='servoHornJ2')
                .add(temp.translate((0,0,0)), name='servoHornJ3')
            )

        (
            robotAsm
            .constrain('j0?servoJ1AxisPlane', 'servoHornJ1?axisPlane', 'Plane')
            .constrain('j0@faces@+Y', 'servoHornJ1?rotationAxisPlane', 'Axis')
            .constrain('j1?servoJ2AxisPlane', 'servoHornJ2?axisPlane', 'Plane')
            .constrain('j1@faces@+X', 'servoHornJ2?rotationAxisPlane', 'Axis')
            .constrain('j3?servoJ3AxisPlane', 'servoHornJ3?axisPlane', 'Plane')
            .constrain('j3@faces@-Z', 'servoHornJ3?rotationAxisPlane', 'Axis')
        )

    if True:
        temp0 = createBrushedServoClip()
        temp1 = createMicroServoClip()
        robotAsm = (
                robotAsm
                .add(temp0, name='clipJ1', color=colors[1])
                .add(temp0.translate((0,0,0)), name='clipJ2', color=colors[1])
                .add(temp0.translate((0,0,0)), name='clipJ3', color=colors[1])
                .add(temp1, name='clipJ4', color=colors[1])
                .add(temp1.translate((0,0,0)), name='clipJ5', color=colors[1])
                .add(temp1.translate((0,0,0)), name='clipJ6', color=colors[1])
            )

        (
            robotAsm
            .constrain('j1?toAxisPlane', 'clipJ1?axisPlane', 'Plane')
            .constrain('j2?toAxisPlane', 'clipJ2?axisPlane', 'Plane')
            .constrain('j3?toAxisPlane', 'clipJ3?axisPlane', 'Plane')
            .constrain('j4?toAxisPlane', 'clipJ4?axisPlane', 'Plane')
            .constrain('j5?toAxisPlane', 'clipJ5?axisPlane', 'Plane')
            .constrain('j6?toAxisPlane', 'clipJ6?axisPlane', 'Plane')
        )

    if True:
        temp = createMicroServoAxisClamp()
        robotAsm = (
                robotAsm
                .add(temp, name='clampJ4', color=colors[1])
                .add(temp.translate((0,0,0)), name='clampJ5', color=colors[1])
                .add(temp.translate((0,0,0)), name='clampJ6', color=colors[1])
            )

        (
            robotAsm
            .constrain('j4?servoJ4AxisPlane', 'clampJ4?axisPlane', 'Plane')
            .constrain('j4@faces@+Z', 'clampJ4?rotationAxisPlane', 'Axis')
            .constrain('j4?servoJ5AxisPlane', 'clampJ5?axisPlane', 'Plane')
            .constrain('j4@faces@+Z', 'clampJ5?rotationAxisPlane', 'Axis')
            .constrain('j6?servoJ6AxisPlane', 'clampJ6?axisPlane', 'Plane')
            .constrain('j6@faces@+Z', 'clampJ6?rotationAxisPlane', 'Axis')
        )

    robotAsm.solve()

    show_object(robotAsm, name='robotAsm')

    return robotAsm

if runCreateRobot:
    result = createRobot()

#result = exporters.export(result, 'cadqueryTest.stl')
