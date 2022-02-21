import serial
import threading
import time
import math
import random
from enum import Enum

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

        self.message = f'Communication error: '
        
        if self.code == CommunicationError.ErrorCode.COULD_NOT_SEND:
            self.message += f'Could not send to port'
        elif self.code == CommunicationError.ErrorCode.NO_RESPONSE:
            self.message += f'No response from node {self.nodeNr}'
        elif (self.code == CommunicationError.ErrorCode.PARTIAL_RESPONSE_TYPE_1 or
                 self.code == CommunicationError.ErrorCode.PARTIAL_RESPONSE_TYPE_2 or
                 self.code == CommunicationError.ErrorCode.PARTIAL_RESPONSE_TYPE_3 or
                 self.code == CommunicationError.ErrorCode.PARTIAL_RESPONSE_TYPE_4):
            self.message += f'Partial response from node {self.nodeNr}, error code {self.code}'
        elif self.code == CommunicationError.ErrorCode.UNEXPECTED_RESPONSE:
            self.message += f'Unexpected response from node {self.nodeNr}'
        elif self.code == CommunicationError.ErrorCode.CHECKSUM_ERROR:
            self.message += f'Checksum error from node {self.nodeNr}'

        super().__init__(self.message)


def removeIntWraparound(newVal, oldVal, bitLenght):
    diff = (newVal - oldVal) % (2**bitLenght)
    if diff > (2**bitLenght / 2):
        diff -= (2**bitLenght)
    newVal = oldVal + diff
    return newVal

class SerialCommunication(object):
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
        self.commandArray.append(int(value))

    def writeInt(self, nr, value):
        int16Value = int(value) % (256 * 256)
        self.commandArray.append(int(nr) + 64)
        self.commandArray.append(int(int16Value % 256))
        self.commandArray.append(int(int16Value / 256))

    def requestReadChar(self, nr):
        self.commandArray.append(int(nr) + 128)
        self.receiveArray.append(int(nr))

    def requestReadInt(self, nr):
        self.commandArray.append(int(nr) + 128 + 64)
        self.receiveArray.append(int(nr) + 64)

    def getLastReadChar(self, nr):
        out = self.charArray[int(nr)]
        if out >= (256 / 2):
            out -= 256
        return out

    def getLastReadInt(self, nr):
        out = self.intArray[int(nr)]
        if out >= (256 * 256 / 2):
            out -= 256 * 256
        return out

    def execute(self):
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
        self.sendBuffer.append(int((checksum) % 256))
        self.sendBuffer.append(int(messageLenght))
        for d in self.commandArray:
            self.sendBuffer.append(int(d))

        bytesSent = self.port.write(bytes(self.sendBuffer))
        if bytesSent != len(self.sendBuffer):
            raise CommunicationError(self.nodeNr, CommunicationError.ErrorCode.COULD_NOT_SEND)

        self.commandArray = []

        receiveArrayCopy = self.receiveArray[:]
        self.receiveArray = []

        c = 0
        error = False
        i = 0
        for d in receiveArrayCopy:
            readBytes = self.port.read()
            if len(readBytes) < 1:
                self.port.read()
                raise CommunicationError(self.nodeNr, CommunicationError.ErrorCode.NO_RESPONSE)
            c = readBytes[0]

            if d == c:
                if d >= 64:
                    readBytes = self.port.read()
                    if len(readBytes) < 1:
                        self.port.read()
                        raise CommunicationError(self.nodeNr, CommunicationError.ErrorCode.PARTIAL_RESPONSE_TYPE_1)
                    c = readBytes[0]
                    value = int(c)

                    readBytes = self.port.read()
                    if len(readBytes) < 1:
                        self.port.read()
                        raise CommunicationError(self.nodeNr, CommunicationError.ErrorCode.PARTIAL_RESPONSE_TYPE_2)
                    c = readBytes[0]
                    value += int(c) * 256;
                    self.intArray[d - 64] = value;
                else:
                    readBytes = self.port.read()
                    if len(readBytes) < 1:
                        self.port.read()
                        raise CommunicationError(self.nodeNr, CommunicationError.ErrorCode.PARTIAL_RESPONSE_TYPE_3)
                    c = readBytes[0]
                    self.charArray[d] = c;

            else:
                while True:
                    readBytes = self.port.read()
                    if len(readBytes) < 1:
                        self.port.read()
                        break
                raise CommunicationError(self.nodeNr, CommunicationError.ErrorCode.UNEXPECTED_RESPONSE)

        readBytes = self.port.read()
        if len(readBytes) < 1:
            self.port.read()
            raise CommunicationError(self.nodeNr, CommunicationError.ErrorCode.PARTIAL_RESPONSE_TYPE_4)
        c = readBytes[0]
        if c != 0xff:
            raise CommunicationError(self.nodeNr, CommunicationError.ErrorCode.CHECKSUM_ERROR)

class SimulateCommunication(SerialCommunication):
    class ServoSim(object):
        def __init__(self):
            self.pos = 1257.5
            self.vel = 0
            self.charArray = [0] * 16
            self.intArray = [0] * 16
            self.intArray[0] = int(self.pos * 32)
            self.intArray[3] = self.intArray[0]
            self.intArray[10] = self.intArray[3]
            self.timestamp = None

        def run(self):
            dt = 0.0
            newTimestamp = time.monotonic()
            if self.timestamp != None:
                dt = newTimestamp - self.timestamp
            self.timestamp = newTimestamp

            force = self.intArray[2]

            if self.charArray[1] == True and dt > 0.0:
                subStep = 10
                for i in range(0, subStep):
                    velInRad = self.vel / 2048 * math.pi

                    damp = 5
                    backEmf = 0.3
                    maxFriction = 40
                    b = 0.1

                    f = force - backEmf * velInRad * abs(force);
                    friction = f - damp * velInRad + velInRad / (dt / subStep * b)
                    if friction > maxFriction:
                        friction = maxFriction
                    elif friction < -maxFriction:
                        friction = -maxFriction

                    self.vel += (dt / subStep * b) * (f - damp * velInRad - friction) / math.pi * 2048

                    self.pos += dt / subStep * self.vel


                self.intArray[12] = int((1 + math.sin(self.pos / 2048 * math.pi * 100)) * 400 + 1000 + random.random() * 10)
                self.intArray[13] = int((1 + math.cos(self.pos / 2048 * math.pi * 100)) * 700 + 1000 + random.random() * 10)

                self.charArray[1] = False
            else:
                newPosRef = self.intArray[0]
                oldPosRef = self.pos * 32

                self.pos = removeIntWraparound(newPosRef, oldPosRef, bitLenght=16) / 32
                self.vel = self.intArray[1]

            if self.charArray[6] == 0:
                backlashSize = int(5 * 32 + random.random() * 10)

                self.intArray[10] = int(round(self.pos * 32))
                if self.intArray[3] < self.intArray[10] - backlashSize:
                    self.intArray[3] = self.intArray[10] - backlashSize
                elif self.intArray[3] > self.intArray[10] + backlashSize:
                    self.intArray[3] = self.intArray[10] + backlashSize
            else:
                self.intArray[3] = int(round(self.pos * 32))
                self.intArray[10] = self.intArray[3]

            self.intArray[4] = int(self.vel)
            self.intArray[5] = int(force)

    def __init__(self):
        super(SimulateCommunication, self).__init__('')
        self.servoSims = []

    def execute(self):
        while len(self.servoSims) < self.nodeNr:
            self.servoSims.append(self.ServoSim())
        servo = self.servoSims[self.nodeNr - 1]
        servo.run()

        i = 0
        while i < len(self.commandArray):
            if self.commandArray[i] >= 128:
                #read request, do nothing in sim
                pass

            elif self.commandArray[i] >= 64:
                intNr = self.commandArray[i] - 64
                i+= 1
                value = self.commandArray[i] % 256
                i+= 1
                value += (self.commandArray[i] % 256) * 256
                if value >= (256 * 256 / 2):
                    value -= 256 * 256
                servo.intArray[intNr] = value
            else:
                charNr = self.commandArray[i]
                i+= 1
                value = self.commandArray[i] % 256
                if value >= (256 / 2):
                    value -= 256
                servo.charArray[charNr] = value

            i+= 1

        self.commandArray = []

        receiveArrayCopy = self.receiveArray[:]
        self.receiveArray = []

        i = 0
        while i < len(receiveArrayCopy):
            if receiveArrayCopy[i] >= 64:
                value = servo.intArray[receiveArrayCopy[i] - 64]
                self.intArray[receiveArrayCopy[i] - 64] = value
            else:
                value = servo.charArray[receiveArrayCopy[i]] % 256
                self.charArray[receiveArrayCopy[i]] = value

            i += 1

class ContinuousValueUpCaster(object):
    def __init__(self, inbutBitLenght):
        self.value = 0
        self.inputBitLen = inbutBitLenght

    def get(self):
        return self.value

    def set(self, v):
        self.value = v

    def update(self, input):
        self.value = removeIntWraparound(input, self.value, self.inputBitLen)

class DCServoCommunicator(object):
    class OpticalEncoderChannelData(object):
        def __init__(self):
            self.a = 0
            self.b = 0
            self.minCostIndex = 0
            self.minCost = 0

    def __init__(self, nodeNr, bus):
        self.activeIntReads = [True] * 16
        self.activeCharReads = [True] * 8
        self.nodeNr = nodeNr;
        self.bus = bus;

        self.communicationIsOk = False
        self.initState = 0
        self.backlashControlDisabled = False
        self.newPositionReference = False
        self.newOpenLoopControlSignal = False

        self.pwmOpenLoopMode = False

        self.controlSpeed = 50
        self.velControlSpeed = 50 * 4
        self.filterSpeed = 50 * 4 * 8
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
        self.activeRefPos = [0] * 5
        self.refVel = 0
        self.feedforwardU = 0
        self.activeFeedforwardU = [0] * 5
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
            self.updateOffset()

    def setControlSpeed(self, controlSpeed, velControlSpeed = None, filterSpeed = None):
        if velControlSpeed == None:
            velControlSpeed = controlSpeed * 4
        if filterSpeed == None:
            filterSpeed = velControlSpeed * 8

        self.controlSpeed = controlSpeed
        self.velControlSpeed = velControlSpeed
        self.filterSpeed = filterSpeed

    def setBacklashControlSpeed(self, backlashCompensationSpeed, backlashCompensationCutOffSpeed, backlashSize):
        self.backlashCompensationSpeed = backlashCompensationSpeed
        self.backlashCompensationSpeedVelDecrease = min(255.0, 255 * 10 / (backlashCompensationCutOffSpeed / abs(self.scale)))
        self.backlashSize = backlashSize / abs(self.scale)

    def setFrictionCompensation(self, fricComp):
        self.frictionCompensation = fricComp

    def disableBacklashControl(self, b = True):
        self.backlashControlDisabled = b

    def isInitComplete(self):
        return self.initState == 10

    def isCommunicationOk(self):
        return self.communicationIsOk

    def setReference(self, pos, vel, feedforwardU):
        self.newPositionReference = True
        self.newOpenLoopControlSignal = False
        self.refPos = (pos - self.offset) / self.scale * self.positionUpscaling
        self.refVel = vel / self.scale

        if self.refVel > 4:
            self.frictionCompensation = abs(self.frictionCompensation)
        elif self.refVel < -4:
            self.frictionCompensation = -abs(self.frictionCompensation)
        self.feedforwardU = feedforwardU + self.frictionCompensation

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
        return self.activeFeedforwardU[2]

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
                pos = self.backlashEncoderPos;
            else:
                self.activeIntReads[10] = True
                self.activeIntReads[11] = True
                pos = self.encoderPos + self.backlashCompensation
        else:
            self.activeIntReads[10] = True
            pos = self.encoderPos

        return self.scale * (self.activeRefPos[2] * (1.0 / self.positionUpscaling) - pos);

    def getCpuLoad(self):
        self.activeIntReads[8] = True
        return self.cpuLoad

    def getLoopTime(self):
        self.activeIntReads[9] = True
        return self.loopTime

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
        self.bus.setNodeNr(self.nodeNr);

        for i, d in enumerate(self.activeIntReads):
            if d:
                self.bus.requestReadInt(i)

        for i, d in enumerate(self.activeCharReads):
            if d:
                self.bus.requestReadChar(i)

        if self.isInitComplete():
            if self.newPositionReference:
                self.bus.writeInt(0, self.refPos % 2**16)
                self.bus.writeInt(1, self.refVel)
                self.bus.writeInt(2, self.feedforwardU)

                self.activeRefPos[4] = self.activeRefPos[3]
                self.activeRefPos[3] = self.activeRefPos[2]
                self.activeRefPos[2] = self.activeRefPos[1]
                self.activeRefPos[1] = self.activeRefPos[0]
                self.activeRefPos[0] = self.refPos

                self.newPositionReference = False
            elif self.newOpenLoopControlSignal:
                self.bus.writeInt(2, self.feedforwardU)
                self.bus.writeChar(1, self.pwmOpenLoopMode)

                self.newOpenLoopControlSignal = False

            self.activeFeedforwardU[4] = self.activeFeedforwardU[3]
            self.activeFeedforwardU[3] = self.activeFeedforwardU[2]
            self.activeFeedforwardU[2] = self.activeFeedforwardU[1]
            self.activeFeedforwardU[1] = self.activeFeedforwardU[0]
            self.activeFeedforwardU[0] = self.feedforwardU;
        else:
            self.bus.writeChar(2, self.backlashControlDisabled)

            self.bus.writeChar(3, self.controlSpeed)
            self.bus.writeChar(4, int(round(self.velControlSpeed / 4.0)))
            self.bus.writeChar(5, int(round(self.filterSpeed / 32.0)))
            self.bus.writeChar(6, self.backlashCompensationSpeed)
            self.bus.writeChar(7, self.backlashCompensationSpeedVelDecrease)
            self.bus.writeChar(8, self.backlashSize)

        self.bus.execute()

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

        if self.isInitComplete():
            self.intReadBufferIndex3Upscaling.update(self.intReadBuffer[3])
            self.intReadBufferIndex10Upscaling.update(self.intReadBuffer[10])
            self.intReadBufferIndex11Upscaling.update(self.intReadBuffer[11])
        else:
            upscaledPos = (self.intReadBuffer[3] % (256 * 256)) + (self.charReadBuffer[9] % 256) * 256 * 256
            if upscaledPos >= 256**3 / 2:
                upscaledPos -= 256**3

            self.intReadBufferIndex3Upscaling.set(upscaledPos)
            self.intReadBufferIndex10Upscaling.set(self.intReadBuffer[10])
            self.intReadBufferIndex11Upscaling.set(self.intReadBuffer[11])
        
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

        if not self.isInitComplete():
            self.initState += 1

            pos = 0.0
            if not self.backlashControlDisabled:
                pos = self.backlashEncoderPos
            else:
                pos = self.encoderPos

            self.activeRefPos[0] = pos * self.positionUpscaling
            self.activeRefPos[1] = self.activeRefPos[0]
            self.activeRefPos[2] = self.activeRefPos[1]
            self.activeRefPos[3] = self.activeRefPos[2]
            self.activeRefPos[4] = self.activeRefPos[3]

            if self.isInitComplete():
                self.updateOffset()

    def updateOffset(self):
        pos = self.getPosition() / self.scale
        self.startPosition /= self.scale

        if pos - self.startPosition > (2048 / 2):
            self.offset -= (4096 / 2) * self.scale
        elif pos - self.startPosition < -(2048 / 2):
            self.offset += (4096 / 2) * self.scale

class ServoManager(object):
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

    def __exit__(self, exc_type, exc_value, exc_traceback):
        self.shutdown()

    def run(self):
        sleepUntilTimePoint = time.time();

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

            except Exception as e:
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

    def setHandlerFunctions(self, newSendCommandHandlerFunction, newReadResultHandlerFunction, newErrorHandlerFunction = None):
        with self.handlerFunctionMutex:
            self.sendCommandHandlerFunction = newSendCommandHandlerFunction
            self.readResultHandlerFunction = newReadResultHandlerFunction
            self.errorHandlerFunction = newErrorHandlerFunction

    def removeHandlerFunctions(self):
        self.setHandlerFunctions(lambda cycleTime, servoManager : cycleTime, lambda cycleTime, servoManager : cycleTime)

    def start(self):
        self.shuttingDown = False

        if self.t != None and threading.current_thread() == self.t:
            return

        if self.t == None or not self.t.is_alive():
            self.t = threading.Thread(target=self.run)
            self.t.start()

    def shutdown(self):
        self.shuttingDown = True

        if self.t != None and threading.current_thread() == self.t:
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

            if e != None:
                raise e
        return not self.shuttingDown

    def getCycleSleepTime(self):
        return self.cycleSleepTime
