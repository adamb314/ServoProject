'''
Module for communicating with ServoProject servos
'''
# pylint: disable=too-many-lines

import threading
import time
import math
import random
from enum import Enum
from dataclasses import dataclass
import serial

pi = 3.1415926535

class CommunicationError(Exception):
    class ErrorCode(Enum):
        COULD_NOT_SEND = 1
        NO_RESPONSE = 2
        PARTIAL_RESPONSE_TYPE_1 = 3
        PARTIAL_RESPONSE_TYPE_2 = 4
        PARTIAL_RESPONSE_TYPE_3 = 5
        PARTIAL_RESPONSE_TYPE_4 = 6
        UNEXPECTED_RESPONSE = 7
        CHECKSUM_ERROR = 8

    def __init__(self, nodeNr, code):
        self.nodeNr = nodeNr
        self.code = code

        self.message = 'Communication error: '

        if self.code == CommunicationError.ErrorCode.COULD_NOT_SEND:
            self.message += 'Could not send to port'
        elif self.code == CommunicationError.ErrorCode.NO_RESPONSE:
            self.message += f'No response from node {self.nodeNr}'
        elif (self.code in (CommunicationError.ErrorCode.PARTIAL_RESPONSE_TYPE_1,
                CommunicationError.ErrorCode.PARTIAL_RESPONSE_TYPE_2,
                CommunicationError.ErrorCode.PARTIAL_RESPONSE_TYPE_3,
                CommunicationError.ErrorCode.PARTIAL_RESPONSE_TYPE_4)):
            self.message += f'Partial response from node {self.nodeNr}, error code {self.code}'
        elif self.code == CommunicationError.ErrorCode.UNEXPECTED_RESPONSE:
            self.message += f'Unexpected response from node {self.nodeNr}'
        elif self.code == CommunicationError.ErrorCode.CHECKSUM_ERROR:
            self.message += f'Checksum error from node {self.nodeNr}'

        super().__init__(self.message)

def removeIntWraparound(newVal, oldVal, bitLenght):
    diff = (newVal - oldVal) % (2**bitLenght)
    if diff > (2**(bitLenght -1)):
        diff -= (2**bitLenght)
    newVal = oldVal + diff
    return newVal

def unsignedToSignedInt(v):
    out = v
    if out >= (2**15):
        out -= 2**16
    return out

def toUnsignedInt16(v):
    return int(v) % (2**16)

def unsignedToSignedChar(v):
    out = v
    if out >= (2**7):
        out -= 2**8
    return out

def toUnsignedChar(v):
    return int(v) % (2**8)

class SimulatedSerialPort:
    def __init__(self):
        self.outStream = bytearray(b'')
        self.inStream = bytearray(b'')

    def write(self, b: bytes) -> int:
        self.outStream += bytearray(b)
        return len(b)

    def read(self, size: int=1) -> bytes:
        out = self.inStream[0:size]
        self.inStream = self.inStream[size:]
        return bytes(out)

    def externalWrite(self, b: bytes) -> int:
        self.inStream += bytearray(b)
        return len(b)

    def externalRead(self, size: int=1) -> bytes:
        out = self.outStream[0:size]
        self.outStream = self.outStream[size:]
        return bytes(out)

class SerialCommunication:
    def __init__(self, devName):
        if devName != '':
            self.port = serial.Serial(devName, 115200, timeout=0.1)

        self.commandArray = []
        self.receiveArray = []

        self.sendBuffer = []

        self.nodeNr = 1
        self.charArray = [0] * 16
        self.intArray = [0] * 16

    def setNodeNr(self, nr):
        self.nodeNr = int(nr)

    def writeChar(self, nr, value):
        self.commandArray.append(int(nr))
        self.commandArray.append(toUnsignedChar(value))

    def writeInt(self, nr, value):
        int16Value = toUnsignedInt16(value)
        self.commandArray.append(int(nr) + 64)
        self.commandArray.append(toUnsignedChar(int16Value))
        self.commandArray.append(toUnsignedChar(int16Value // 2**8))

    def requestReadChar(self, nr):
        self.commandArray.append(int(nr) + 128)
        self.receiveArray.append(int(nr))

    def requestReadInt(self, nr):
        self.commandArray.append(int(nr) + 128 + 64)
        self.receiveArray.append(int(nr) + 64)

    def getLastReadChar(self, nr):
        out = self.charArray[int(nr)]
        return toUnsignedChar(out)

    def getLastReadInt(self, nr):
        out = self.intArray[int(nr)]
        return unsignedToSignedInt(out)

    def execute(self):
        self._executeSend()
        self._executeReceive()

    def _executeSend(self):
        checksum = 0
        messageLenght = 0

        checksum -= self.nodeNr

        i = 0
        while i < len(self.commandArray):
            if self.commandArray[i] >= 128:
                checksum -= self.commandArray[i]
                messageLenght += 1
            elif self.commandArray[i] >= 64:
                checksum -= self.commandArray[i]
                i += 1
                checksum -= self.commandArray[i]
                i += 1
                checksum -= self.commandArray[i]
                messageLenght += 3
            else:
                checksum -= self.commandArray[i]
                i += 1
                checksum -= self.commandArray[i]
                messageLenght += 2
            i += 1

        checksum -= messageLenght

        self.sendBuffer = []
        self.sendBuffer.append(int(self.nodeNr))
        self.sendBuffer.append(toUnsignedChar(checksum))
        self.sendBuffer.append(int(messageLenght))
        for d in self.commandArray:
            self.sendBuffer.append(int(d))

        bytesSent = self.port.write(bytes(self.sendBuffer))
        if bytesSent != len(self.sendBuffer):
            raise CommunicationError(self.nodeNr, CommunicationError.ErrorCode.COULD_NOT_SEND)

        self.commandArray = []

    def _executeReceive(self):
        def decodeChar8(port):
            readBytes = port.read()
            if len(readBytes) < 1:
                port.read()
                raise CommunicationError(self.nodeNr, CommunicationError.ErrorCode.PARTIAL_RESPONSE_TYPE_3)
            return readBytes[0]

        def decodeInt16(port):
            readBytes = port.read()
            if len(readBytes) < 1:
                port.read()
                raise CommunicationError(self.nodeNr, CommunicationError.ErrorCode.PARTIAL_RESPONSE_TYPE_1)
            c = readBytes[0]
            value = int(c)

            readBytes = port.read()
            if len(readBytes) < 1:
                port.read()
                raise CommunicationError(self.nodeNr, CommunicationError.ErrorCode.PARTIAL_RESPONSE_TYPE_2)
            c = readBytes[0]
            value += int(c) * 2**8

            return value

        def emptyReadBuffer(port):
            while True:
                readBytes = port.read()
                if len(readBytes) < 1:
                    port.read()
                    break

        receiveArrayCopy = self.receiveArray[:]
        self.receiveArray = []

        c = 0
        for d in receiveArrayCopy:
            readBytes = self.port.read()
            if len(readBytes) < 1:
                self.port.read()
                raise CommunicationError(self.nodeNr, CommunicationError.ErrorCode.NO_RESPONSE)
            c = readBytes[0]

            if d == c:
                if d >= 64:
                    self.intArray[d - 64] = decodeInt16(self.port)
                else:
                    self.charArray[d] = decodeChar8(self.port)

            else:
                emptyReadBuffer(self.port)
                raise CommunicationError(self.nodeNr, CommunicationError.ErrorCode.UNEXPECTED_RESPONSE)

        readBytes = self.port.read()
        if len(readBytes) < 1:
            self.port.read()
            raise CommunicationError(self.nodeNr, CommunicationError.ErrorCode.PARTIAL_RESPONSE_TYPE_4)
        c = readBytes[0]
        if c != 0xff:
            raise CommunicationError(self.nodeNr, CommunicationError.ErrorCode.CHECKSUM_ERROR)

class ComDelayInt:
    def __init__(self, delay: int, initValue: int = 0):
        self.delayArray = [0] * delay
        self.comDir = 0
        self.reset(initValue)

    def reset(self, value: int):
        for i, _ in enumerate(self.delayArray):
            self.delayArray[i] = value

    def getLeft(self) -> int:
        return self.delayArray[0]

    def setLeft(self, v: int):
        self.comDir = 1
        self.delayArray[0] = v

    def getRight(self) -> int:
        return self.delayArray[-1]

    def setRight(self, v: int):
        self.comDir = -1
        self.delayArray[-1] = v

    def execute(self):
        d = self.delayArray
        if self.comDir == 1:
            for i in range(len(d) - 2, -1, -1):
                d[i + 1] = d[i]
        elif self.comDir == -1:
            for i in range(0, len(d) - 1):
                d[i] = d[i + 1]

class SimulateCommunication(SerialCommunication):
    # pylint: disable=too-many-instance-attributes
    class ServoSim:
        def __init__(self, nodeNr, enableNoise=True):
            self.nodeNr = nodeNr
            self.pos = 1257.5
            self.vel = 0
            self.loopNr = 0
            self.charArray = [0] * 16
            self.intArray = [0] * 16

            upscaledPos = int(round(self.pos * 32))
            self.comDelayedPos = ComDelayInt(delay=2, initValue=upscaledPos)
            self.comDelayedVel = ComDelayInt(delay=2, initValue=self.intArray[4])
            self.comDelayedForce = ComDelayInt(delay=2, initValue=self.intArray[5])
            self.comDelayedLoopNr = ComDelayInt(delay=2, initValue=self.loopNr)

            self.activeSimForce = self.comDelayedForce.getLeft()

            self.intArray[3] = toUnsignedInt16(self.comDelayedPos.getLeft())
            self.intArray[10] = self.intArray[3]
            self.charArray[9] = toUnsignedChar(self.comDelayedPos.getLeft() // 2**16)

            self.intArray[0] = self.intArray[3]
            self.timestamp = None
            self.enableNoise = enableNoise
            self.startTimeStamp = None

        def run(self):
            # pylint: disable=too-many-locals, too-many-statements, too-many-branches
            gearRatio = 100
            damp = 10
            backEmf = 0.3
            maxFriction = 75
            b = 0.1
            pwmOffset = -575
            servoLoopCycleTime = 0.0006

            time.sleep(servoLoopCycleTime)

            dt = 0.0
            newTimestamp = time.monotonic()
            if self.timestamp is not None:
                dt = newTimestamp - self.timestamp
            self.timestamp = newTimestamp

            if self.startTimeStamp is None:
                self.startTimeStamp = newTimestamp

            newLoopNr = int(round(((newTimestamp - self.startTimeStamp) / servoLoopCycleTime)))
            nrOfLoops = newLoopNr - self.loopNr
            self.loopNr = newLoopNr

            dt = nrOfLoops * servoLoopCycleTime

            rawPwmMode = self.charArray[1] != 0

            force = 0

            if dt == 0.0:
                pass
            elif rawPwmMode:
                force = self.activeSimForce

                forceSign = 1 if force >= 0 else -1
                force = max(0.0, (abs(force) + pwmOffset) * 1023 / (1023 + pwmOffset)) * forceSign

                subStep = 1000
                for _ in range(0, subStep):
                    velInRad = self.vel / 2048 * math.pi

                    f = force
                    f -= backEmf * velInRad * abs(self.activeSimForce)
                    friction = f - damp * velInRad + velInRad / (dt / subStep * b)
                    if friction > maxFriction:
                        friction = maxFriction
                    elif friction < -maxFriction:
                        friction = -maxFriction

                    self.vel += (dt / subStep * b) * (f - damp * velInRad - friction) / math.pi * 2048

                    self.pos += dt / subStep * self.vel

                self.charArray[1] = False
            else:
                newPosRef = self.intArray[0]
                oldPosRef = self.pos * 32

                self.pos = removeIntWraparound(newPosRef, oldPosRef, bitLenght=16) / 32
                newVel = unsignedToSignedInt(self.intArray[1])
                force = (newVel - self.vel) / (dt * b / math.pi * 2048)
                force += 50 * math.sin(self.pos * gearRatio * 6 / 2048 * math.pi)
                force = min(max(force, -1023), 1023)
                self.vel = newVel

            self.activeSimForce = unsignedToSignedInt(self.intArray[2])

            self.comDelayedPos.execute()
            self.comDelayedVel.execute()
            self.comDelayedForce.execute()
            self.comDelayedLoopNr.execute()

            posNoise = 0.0
            velNoise = 0.0
            if self.enableNoise:
                posNoise = (2.0 * random.random() - 1.0) * abs(self.vel) * 0.001
                velNoise = 0.03 * (2.0 * random.random() - 1.0)
            self.comDelayedPos.setRight(int(round(self.pos * 32 + posNoise)))
            self.comDelayedVel.setRight(toUnsignedInt16(round(self.vel * (1.0 + velNoise))))
            self.comDelayedForce.setRight(toUnsignedInt16(round(force)))
            self.comDelayedLoopNr.setRight(self.loopNr % 256)

            # optical encoder simulation
            optEncPos = self.comDelayedPos.getLeft() / 32 * gearRatio
            optEncChA = ((1 + math.sin(optEncPos / 2048 * math.pi)) * 400
                                        + 1000 + random.random() * 10)
            optEncChB = ((1 + math.cos(optEncPos / 2048 * math.pi)) * 700 + 1000
                                        + random.random() * 10)
            self.intArray[12] = toUnsignedInt16(round(optEncChA))
            self.intArray[13] = toUnsignedInt16(round(optEncChB))
            self.intArray[14] = toUnsignedInt16(round(optEncPos * 2048 / 4096)) % 2048

            self.charArray[9] = toUnsignedChar(self.comDelayedPos.getLeft() // 2**16)

            if self.charArray[6] == 0:
                backlashSize = int(5 * 32 + random.random() * 10)

                self.intArray[10] = toUnsignedInt16(self.comDelayedPos.getLeft())
                if self.intArray[3] < self.intArray[10] - backlashSize:
                    self.intArray[3] = self.intArray[10] - backlashSize
                elif self.intArray[3] > self.intArray[10] + backlashSize:
                    self.intArray[3] = self.intArray[10] + backlashSize
            else:
                self.intArray[3] = toUnsignedInt16(self.comDelayedPos.getLeft())
                self.intArray[10] = self.intArray[3]

            self.intArray[4] = self.comDelayedVel.getLeft()
            self.intArray[5] = self.comDelayedForce.getLeft()
            self.intArray[7] = self.intArray[5]
            self.charArray[11] = self.comDelayedLoopNr.getLeft()

        def handleCommunication(self, port: SimulatedSerialPort):
            #pylint: disable=too-many-branches, too-many-statements
            messageNodeNr = port.externalRead()[0]

            if self.nodeNr != messageNodeNr:
                raise Exception('nodeNr error in simulation')

            intArrayBuffer = self.intArray[:]
            charArrayBuffer = self.charArray[:]

            sendCommandBuffer = []

            receiveCompleate = False
            communicationError = False

            checksum = port.externalRead()[0]
            checksum += self.nodeNr

            messageLength = port.externalRead()[0]

            checksum += messageLength

            while messageLength != 0:
                command = port.externalRead()[0]
                messageLength -= 1

                checksum += command

                if (command >> 7) == 1:
                    sendCommandBuffer.append(command - 128)

                    if messageLength == 0:
                        receiveCompleate = True
                        break

                elif (command >> 6) == 1:
                    if messageLength == 1:
                        port.externalRead()
                        communicationError = True

                    byteValue = port.externalRead()[0]
                    value = byteValue
                    checksum += byteValue

                    byteValue = port.externalRead()[0]
                    value += byteValue * 2**8
                    checksum += byteValue

                    i = command - 64
                    if i < len(intArrayBuffer):
                        intArrayBuffer[i] = value
                    else:
                        communicationError = True

                    messageLength -= 2

                    if messageLength == 0:
                        receiveCompleate = True
                        break
                else:
                    if command < len(charArrayBuffer):
                        byteValue = port.externalRead()[0]

                        charArrayBuffer[command] = byteValue
                        checksum += byteValue
                    else:
                        port.externalRead()
                        communicationError = True

                    messageLength -= 1

                    if messageLength == 0:
                        receiveCompleate = True
                        break

                if messageLength == 0:
                    communicationError = True

            for sendCommand in sendCommandBuffer:
                if (sendCommand >> 6) == 1:
                    value = 0
                    i = sendCommand - 64
                    if i < len(self.intArray):
                        value = self.intArray[i]

                    port.externalWrite(sendCommand.to_bytes(1, 'little'))
                    port.externalWrite(value.to_bytes(2, 'little'))
                else:
                    value = 0
                    if sendCommand < len(self.charArray):
                        value = self.charArray[sendCommand]

                    port.externalWrite(sendCommand.to_bytes(1, 'little'))
                    port.externalWrite(value.to_bytes(1, 'little'))

            checksum = toUnsignedChar(checksum)

            if checksum == 0 and not communicationError:
                port.externalWrite(int(255).to_bytes(1, 'little'))
            else:
                port.externalWrite(int(0).to_bytes(1, 'little'))

            if receiveCompleate and checksum == 0:
                self.intArray = intArrayBuffer
                self.charArray = charArrayBuffer

    def __init__(self, enableNoise=True):
        super().__init__('')
        self.port = SimulatedSerialPort()
        self.servoSims = []
        self.enableNoise = enableNoise

    def execute(self):
        while len(self.servoSims) < self.nodeNr:
            newServo = self.ServoSim(self.nodeNr, self.enableNoise)
            self.servoSims.append(newServo)
        servo = self.servoSims[self.nodeNr - 1]
        servo.run()

        self._executeSend()

        servo.handleCommunication(self.port)

        self._executeReceive()

class ContinuousValueUpCaster:
    def __init__(self, inbutBitLenght):
        self.value = 0
        self.inputBitLen = inbutBitLenght

    def get(self):
        return self.value

    def set(self, v):
        self.value = v

    def update(self, v):
        self.value = removeIntWraparound(v, self.value, self.inputBitLen)

class DCServoCommunicator:
    # pylint: disable=too-many-instance-attributes, too-many-public-methods
    @dataclass
    class OpticalEncoderChannelData:
        a: int = 0
        b: int = 0
        minCostIndex: int = 0
        minCost: int = 0

    class ControlLoopSyncedTimeHandler:
        us200 = 200 / 1000000

        def __init__(self):
            self.initDataList = []
            self.loopCycleTime = None
            self.lastRemoteTime = 0

        def isInitialized(self):
            return self.loopCycleTime is not None

        def initialize(self, loopNr):
            if self.isInitialized():
                return True

            if len(self.initDataList) < 20:
                self.initDataList.append([time.time(), loopNr])
                return False

            listOfDt = []
            for d in zip(self.initDataList[1:], self.initDataList[0:-1]):
                localTimeDiff = d[0][0] - d[1][0]
                loopNrDiff = (d[0][1] - d[1][1] + 128) % 256 - 128

                if loopNrDiff == 0:
                    continue

                listOfDt.append(localTimeDiff / loopNrDiff)

            temp = sorted(listOfDt)[len(listOfDt)//4:-len(listOfDt)//4]
            if len(temp) == 0:
                self.loopCycleTime = -1
                return True

            self.loopCycleTime = sum(temp) / len(temp)
            self.loopCycleTime = round(self.loopCycleTime / self.us200) * self.us200
            return True

        def update(self, loopNr):
            localTime = time.time()

            if self.loopCycleTime == -1:
                self.lastRemoteTime = localTime
                return

            localTimeDiff = localTime - self.initDataList[-1][0]

            nrOfLoops = ((loopNr - self.initDataList[-1][1]) % 256)
            nrOfLoops += round((localTimeDiff / self.loopCycleTime - nrOfLoops) / 256) * 256

            self.lastRemoteTime += nrOfLoops * self.loopCycleTime

            self.initDataList[-1][0] = localTime
            self.initDataList[-1][1] = loopNr

        def get(self):
            return self.lastRemoteTime

    def __init__(self, nodeNr, bus):
        self.activeIntReads = [True] * 16
        self.activeCharReads = [True] * 16
        self.nodeNr = nodeNr
        self.bus = bus

        self.communicationIsOk = False
        self.initState = 0
        self.remoteTimeHandler = self.ControlLoopSyncedTimeHandler()
        self.backlashControlDisabled = False
        self.newPositionReference = False
        self.newOpenLoopControlSignal = False

        self.pwmOpenLoopMode = False

        self.controlSpeed = 50
        self.velControlSpeed = 50 * 4
        self.filterSpeed = 50 * 4 * 8
        self.inertiaMarg = 1.0
        self.backlashCompensationSpeed = 10
        self.backlashCompensationSpeedVelDecrease = 0
        self.backlashSize = 0

        self.intReadBuffer = [0] * 16
        self.charReadBuffer = [0] * 16

        self.intReadBufferIndex3Upscaling = ContinuousValueUpCaster(16)
        self.intReadBufferIndex10Upscaling = ContinuousValueUpCaster(16)
        self.intReadBufferIndex11Upscaling = ContinuousValueUpCaster(16)

        self.backlashEncoderPos = 0.0
        self.encoderPos = 0.0
        self.backlashCompensation = 0.0
        self.encoderVel = 0
        self.controlSignal = 0
        self.current = 0
        self.pwmControlSignal = 0
        self.cpuLoad = 0
        self.loopTime = 0
        self.opticalEncoderChannelData = self.OpticalEncoderChannelData()

        self.refPos = 0
        self.comDelayRefPos = ComDelayInt(delay=3, initValue=0)
        self.refVel = 0
        self.feedforwardU = 0
        self.comDelayFeedforwardU = ComDelayInt(delay=3, initValue=0)
        self.frictionCompensation = 0.0

        self.offset = 0.0
        self.startPosition = 0.0
        self.scale = 1.0

        self.positionUpscaling = 32

    def setOffsetAndScaling(self, scale, offset, startPosition = 0):
        self.scale = scale
        self.offset = offset
        self.startPosition = startPosition

        if self.isInitComplete():
            self._updateOffset()

    def setControlSpeed(self, controlSpeed, velControlSpeed = None, filterSpeed = None, inertiaMarg = 1.0):
        if velControlSpeed is None:
            velControlSpeed = controlSpeed * 4
        if filterSpeed is None:
            filterSpeed = velControlSpeed * 8

        self.controlSpeed = controlSpeed
        self.velControlSpeed = velControlSpeed
        self.filterSpeed = filterSpeed
        self.inertiaMarg = min(max(inertiaMarg, 1.0), 1 + 255.0 / 128)

    def setBacklashControlSpeed(self, backlashCompensationSpeed, backlashCompensationCutOffSpeed, backlashSize):
        self.backlashCompensationSpeed = backlashCompensationSpeed
        self.backlashCompensationSpeedVelDecrease = min(
                255.0, 255 * 10 / (backlashCompensationCutOffSpeed / abs(self.scale)))
        self.backlashSize = backlashSize / abs(self.scale)

    def setFrictionCompensation(self, fricComp):
        self.frictionCompensation = fricComp

    def disableBacklashControl(self, b = True):
        self.backlashControlDisabled = b

    def isInitComplete(self):
        return self.initState >= 10 and self.remoteTimeHandler.isInitialized()

    def isCommunicationOk(self):
        return self.communicationIsOk

    def setReference(self, pos, vel, feedforwardU):
        self.newPositionReference = True
        self.newOpenLoopControlSignal = False
        self.refPos = round((pos - self.offset) / self.scale * self.positionUpscaling)
        self.refVel = round(vel / self.scale)
        self.refVel = max(1, abs(self.refVel)) * (1 if self.refVel > 0 else (-1 if self.refVel < 0 else 0))

        if self.refVel > 4:
            self.frictionCompensation = abs(self.frictionCompensation)
        elif self.refVel < -4:
            self.frictionCompensation = -abs(self.frictionCompensation)
        self.feedforwardU = round(feedforwardU + self.frictionCompensation)

    def setOpenLoopControlSignal(self, feedforwardU, pwmMode):
        self.newOpenLoopControlSignal = True
        self.newPositionReference = False
        self.pwmOpenLoopMode = pwmMode
        self.feedforwardU = feedforwardU

    def getPosition(self, withBacklash = True):
        pos = 0.0
        if withBacklash and not self.backlashControlDisabled:
            self.activeIntReads[3] = True
            pos = self.backlashEncoderPos
        else:
            self.activeIntReads[10] = True
            pos = self.encoderPos

        return self.scale * pos + self.offset

    def getVelocity(self):
        self.activeIntReads[4] = True
        return self.scale * self.encoderVel

    def getControlSignal(self):
        self.activeIntReads[5] = True
        return self.controlSignal

    def getFeedforwardU(self):
        return self.comDelayFeedforwardU.getRight()

    def getCurrent(self):
        self.activeIntReads[6] = True
        return self.current

    def getPwmControlSignal(self):
        self.activeIntReads[7] = True
        return self.pwmControlSignal

    def getControlError(self, withBacklash = True):
        pos = 0.0
        if not self.backlashControlDisabled:
            if withBacklash:
                self.activeIntReads[3] = True
                pos = self.backlashEncoderPos
            else:
                self.activeIntReads[10] = True
                self.activeIntReads[11] = True
                pos = self.encoderPos + self.backlashCompensation
        else:
            self.activeIntReads[10] = True
            pos = self.encoderPos

        return self.scale * (self.comDelayRefPos.getRight() * (1.0 / self.positionUpscaling) - pos)

    def getCpuLoad(self):
        self.activeIntReads[8] = True
        return self.cpuLoad

    def getLoopTime(self):
        self.activeIntReads[9] = True
        return self.loopTime

    def getTime(self):
        self.activeCharReads[11] = True
        return self.remoteTimeHandler.get()

    def getBacklashCompensation(self):
        self.activeIntReads[11] = True
        return self.scale * self.backlashCompensation

    def getOpticalEncoderChannelData(self):
        self.activeIntReads[12] = True
        self.activeIntReads[13] = True
        self.activeIntReads[14] = True
        self.activeIntReads[15] = True
        return self.opticalEncoderChannelData

    def getScaling(self):
        return self.scale

    def getOffset(self):
        return self.offset

    def run(self):
        #pylint: disable=too-many-branches, too-many-statements

        self.bus.setNodeNr(self.nodeNr)

        # handle read requests
        for i, d in enumerate(self.activeIntReads):
            if d:
                self.bus.requestReadInt(i)

        for i, d in enumerate(self.activeCharReads):
            if d:
                self.bus.requestReadChar(i)

        loopNrReadActive = self.activeCharReads[11]

        # handle writes
        if self.isInitComplete():
            if self.newPositionReference:
                self.bus.writeInt(0, toUnsignedInt16(self.refPos))
                self.bus.writeInt(1, self.refVel)
                self.bus.writeInt(2, self.feedforwardU)

                self.comDelayRefPos.execute()
                self.comDelayRefPos.setLeft(self.refPos)

                self.newPositionReference = False
            elif self.newOpenLoopControlSignal:
                self.bus.writeInt(2, self.feedforwardU)
                self.bus.writeChar(1, self.pwmOpenLoopMode)

                self.newOpenLoopControlSignal = False

            self.comDelayFeedforwardU.execute()
            self.comDelayFeedforwardU.setLeft(self.feedforwardU)
        else:
            self.bus.writeChar(2, self.backlashControlDisabled)

            self.bus.writeChar(3, self.controlSpeed)
            self.bus.writeChar(4, int(round(self.velControlSpeed / 4.0)))
            self.bus.writeChar(5, int(round(self.filterSpeed / 32.0)))
            self.bus.writeChar(10, int(round((self.inertiaMarg - 1.0) * 128.0)))
            self.bus.writeChar(6, self.backlashCompensationSpeed)
            self.bus.writeChar(7, self.backlashCompensationSpeedVelDecrease)
            self.bus.writeChar(8, self.backlashSize)

        self.bus.execute()

        # handle requested reads
        for i, d in enumerate(self.activeIntReads):
            if d:
                if self.isInitComplete():
                    self.activeIntReads[i] = False
                self.intReadBuffer[i] = self.bus.getLastReadInt(i)

        for i, d in enumerate(self.activeCharReads):
            if d:
                if self.isInitComplete():
                    self.activeCharReads[i] = False
                self.charReadBuffer[i] = self.bus.getLastReadChar(i)

        # update members
        if self.isInitComplete():
            self.intReadBufferIndex3Upscaling.update(self.intReadBuffer[3])
            self.intReadBufferIndex10Upscaling.update(self.intReadBuffer[10])
            self.intReadBufferIndex11Upscaling.update(self.intReadBuffer[11])

            if loopNrReadActive:
                self.remoteTimeHandler.update(self.charReadBuffer[11])
        else:
            upscaledPos = toUnsignedInt16(self.intReadBuffer[3]) + toUnsignedChar(self.charReadBuffer[9]) * 2**16
            if upscaledPos >= 2**23:
                upscaledPos -= 2**24

            encPosWithBacklashComp = self.intReadBuffer[10] + self.intReadBuffer[11]
            overflowedPart = ((upscaledPos - encPosWithBacklashComp) // 2**16) * 2**16

            self.intReadBufferIndex3Upscaling.set(upscaledPos)
            self.intReadBufferIndex10Upscaling.set(self.intReadBuffer[10])
            self.intReadBufferIndex11Upscaling.set(self.intReadBuffer[11] + overflowedPart)

        self.backlashEncoderPos = self.intReadBufferIndex3Upscaling.get() * (1.0 / self.positionUpscaling)
        self.encoderPos = self.intReadBufferIndex10Upscaling.get() * (1.0 / self.positionUpscaling)
        self.backlashCompensation = self.intReadBufferIndex11Upscaling.get() * (1.0 / self.positionUpscaling)

        self.encoderVel = self.intReadBuffer[4]
        self.controlSignal = self.intReadBuffer[5]
        self.current = self.intReadBuffer[6]
        self.pwmControlSignal = self.intReadBuffer[7]
        self.cpuLoad = self.intReadBuffer[8]
        self.loopTime = self.intReadBuffer[9]
        self.opticalEncoderChannelData.a = self.intReadBuffer[12]
        self.opticalEncoderChannelData.b = self.intReadBuffer[13]
        self.opticalEncoderChannelData.minCostIndex = self.intReadBuffer[14]
        self.opticalEncoderChannelData.minCost = self.intReadBuffer[15]

        # handle initialization
        if not self.isInitComplete():
            self.initState += 1

            self.remoteTimeHandler.initialize(self.charReadBuffer[11])

            pos = 0.0
            if not self.backlashControlDisabled:
                pos = self.backlashEncoderPos
            else:
                pos = self.encoderPos

            self.comDelayRefPos.reset(pos * self.positionUpscaling)

            if self.isInitComplete():
                self._updateOffset()

    def _updateOffset(self):
        pos = self.getPosition() / self.scale
        self.startPosition /= self.scale

        wrapSize = 2**24 / self.positionUpscaling

        if pos - self.startPosition > wrapSize / 2:
            self.offset -= wrapSize * self.scale
        elif pos - self.startPosition < -wrapSize / 2:
            self.offset += wrapSize * self.scale

class ServoManager:
    def __init__(self, cycleTime, initFunction):
        self.cycleTime = cycleTime

        self.cycleSleepTime = 0

        self.shuttingDown = True

        self.exception = None

        self.t = None

        self.handlerFunctionMutex = threading.Lock()
        self.sendCommandHandlerFunction = None
        self.readResultHandlerFunction = None
        self.errorHandlerFunction = None

        self.servoArray = initFunction()

        while True:
            allDone = True
            for s in self.servoArray:
                allDone = allDone and s.isInitComplete()
                s.run()

            if allDone:
                break

        self.currentPosition = []
        for s in self.servoArray:
            self.currentPosition.append(s.getPosition())

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, excType, excValue, excTraceback):
        self.shutdown()

    def run(self):
        sleepUntilTimePoint = time.time()

        while not self.shuttingDown:
            try:
                self.cycleSleepTime = sleepUntilTimePoint - time.time()
                time.sleep(max(0, sleepUntilTimePoint - time.time()))
                sleepUntilTimePoint += self.cycleTime

                with self.handlerFunctionMutex:
                    tempSendHandlerFunction = self.sendCommandHandlerFunction
                    tempReadHandlerFunction = self.readResultHandlerFunction

                if tempSendHandlerFunction:
                    tempSendHandlerFunction(self.cycleTime, self)

                for s in self.servoArray:
                    s.run()

                for i, s in enumerate(self.servoArray):
                    self.currentPosition[i] = s.getPosition()

                if tempReadHandlerFunction:
                    tempReadHandlerFunction(self.cycleTime, self)

            except Exception as e:  # pylint: disable=broad-except
                with self.handlerFunctionMutex:
                    tempErrorHandlerFunction = self.errorHandlerFunction

                if tempErrorHandlerFunction:
                    self.shutdown()
                    tempErrorHandlerFunction(e)
                else:
                    self.exception = e
                    self.shutdown()
                    raise e

    def getPosition(self):
        return self.currentPosition

    def setHandlerFunctions(self, newSendCommandHandlerFunction,
                            newReadResultHandlerFunction, newErrorHandlerFunction = None):
        with self.handlerFunctionMutex:
            self.sendCommandHandlerFunction = newSendCommandHandlerFunction
            self.readResultHandlerFunction = newReadResultHandlerFunction
            self.errorHandlerFunction = newErrorHandlerFunction

    def removeHandlerFunctions(self):
        self.setHandlerFunctions(lambda cycleTime, servoManager : cycleTime, lambda cycleTime, servoManager : cycleTime)

    def start(self):
        self.shuttingDown = False

        if self.t is not None and threading.current_thread() == self.t:
            return

        if self.t is None or not self.t.is_alive():
            self.t = threading.Thread(target=self.run)
            self.t.start()

    def shutdown(self):
        self.shuttingDown = True

        if self.t is not None and threading.current_thread() == self.t:
            return

        self.t.join()

    def registerUnhandledException(self, e):
        with self.handlerFunctionMutex:
            self.exception = e

    def getUnhandledException(self):
        with self.handlerFunctionMutex:
            e = self.exception
            self.exception = None
            return e

    def isAlive(self, raiseException = True):
        if raiseException:
            e = None
            if self.shuttingDown:
                e = self.getUnhandledException()

            if e is not None:
                raise e
        return not self.shuttingDown

    def getCycleSleepTime(self):
        return self.cycleSleepTime
