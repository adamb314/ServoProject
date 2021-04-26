#!/bin/python3
import numpy as np
import scipy.signal
import math
import random
import matplotlib.pyplot as plt
from matplotlib.backends.backend_gtk3agg import (
    FigureCanvasGTK3Agg as FigureCanvas)
from matplotlib.figure import Figure

import numba
import serial
import serial.tools.list_ports
import threading
import time

class SerialCommunication(object):
    def __init__(self, devName):
        if devName != '':
            self.port = serial.Serial(devName, 115200, timeout=0.1)

        self.commandArray = []
        self.receiveArray = []

        self.sendBuffer = []

        self.nodeNr = 1
        self.charArray = [0] * 8
        self.intArray = [0] * 16

    def setNodeNr(self, nr):
        self.nodeNr = int(nr)

    def writeChar(self, nr, value):
        self.commandArray.append(int(nr))
        self.commandArray.append(int(value))

    def writeInt(self, nr, value):
        value = value % (256 * 256)
        self.commandArray.append(int(nr) + 64)
        self.commandArray.append(int(value % 256))
        self.commandArray.append(int(value / 256))

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
            raise Exception('all data not sent error')

        self.commandArray = []

        receiveArrayCopy = self.receiveArray[:]
        self.receiveArray = []

        c = 0
        error = False
        i = 0
        for d in receiveArrayCopy:
            readBytes = self.port.read()
            if len(readBytes) < 1:
                print("error1\n")
                self.port.read()
                return False
            c = readBytes[0]

            if d == c:
                if d >= 64:
                    readBytes = self.port.read()
                    if len(readBytes) < 1:
                        print("error2\n")
                        self.port.read()
                        return False
                    c = readBytes[0]
                    value = int(c)

                    readBytes = self.port.read()
                    if len(readBytes) < 1:
                        print("error3\n")
                        self.port.read()
                        return False
                    c = readBytes[0]
                    value += int(c) * 256;
                    self.intArray[d - 64] = value;
                else:
                    readBytes = self.port.read()
                    if len(readBytes) < 1:
                        print("error4\n")
                        self.port.read()
                        return False
                    c = readBytes[0]
                    self.charArray[d] = c;

            else:
                print("error5\n")

                while True:
                    readBytes = self.port.read()
                    if len(readBytes) < 1:
                        self.port.read()
                        break

        readBytes = self.port.read()
        if len(readBytes) < 1:
            self.port.read()
            return False
        c = readBytes[0]
        if c != 0xff:
            return False

        return True

class SimulateCommunication(SerialCommunication):
    class ServoSim(object):
        def __init__(self):
            self.pos = 40240
            self.vel = 0
            self.charArray = [0] * 8
            self.intArray = [0] * 16
            self.intArray[3] = self.pos

        def run(self):
            force = self.intArray[2]

            if self.charArray[1] == True:
                dt = 0.004

                damp = 5
                friction = force - damp * self.vel + self.vel / dt
                if friction > 10:
                    friction = 10
                elif friction < -10:
                    friction = -10

                self.vel = self.vel + dt * (force - damp * self.vel - friction)

                self.pos += dt * self.vel

                self.intArray[12] = (1 + math.sin(self.pos)) * 1000 + 1000 + random.random() * 15
                self.intArray[13] = (1 + math.cos(self.pos)) * 1000 + 1000 + random.random() * 15

                self.intArray[3] = self.pos
                self.intArray[10] = self.pos
                self.intArray[4] = self.vel
            else:
                self.pos = self.intArray[0]
                self.vel = self.intArray[1]
                self.intArray[3] = self.pos
                self.intArray[10] = self.pos
                self.intArray[4] = self.vel

            self.intArray[5] = force

    def __init__(self):
        super(SimulateCommunication, self).__init__('')
        self.servoSims = []

        for i in range(0, 7):
            self.servoSims.append(self.ServoSim())

    def execute(self):
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

        return True

class ContinuousValueUpCaster(object):
    def __init__(self, inbutBitLenght):
        self.value = 0
        self.inputBitLen = inbutBitLenght

    def get(self):
        return self.value

    def set(self, v):
        self.value = v

    def update(self, input):
        diff = (input - self.value) % (2**self.inputBitLen)
        if diff > 2**self.inputBitLen / 2:
            diff -= 2**self.inputBitLen

        self.value += diff


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
        self.backlashCompensationSpeed = 10
        self.backlashCompensationSpeedVelDecrease = 0
        self.backlashSize = 0

        self.intReadBuffer = [0] * 16
        self.charReadBuffer = [0] * 8

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

    def setControlSpeed(self, controlSpeed):
        self.controlSpeed = controlSpeed

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
            self.bus.writeChar(4, self.backlashCompensationSpeed)
            self.bus.writeChar(5, self.backlashCompensationSpeedVelDecrease)
            self.bus.writeChar(6, self.backlashSize)

        self.communicationIsOk = self.bus.execute()

        if self.communicationIsOk:
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
                upscaledPos = (self.intReadBuffer[3] % (256 * 256)) + (self.charReadBuffer[7] % 256) * 256 * 256
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

pi = 3.1415926535

class Robot(object):
    def __init__(self, communication, simulate = [False] * 7, cycleTime = 0.018, initFunction=lambda robot: robot):
        self.communicationSim = SimulateCommunication()

        self.dcServoArray = []

        for i, sim in enumerate(simulate[0:6]):
            com = communication
            if sim:
                com = self.communicationSim

            self.dcServoArray.append(DCServoCommunicator(1 + i, com))

        com = communication
        if simulate[6]:
            com = self.communicationSim
        self.gripperServo = DCServoCommunicator(7, com)
        
        self.cycleTime = cycleTime

        self.dof = 6

        self.cycleSleepTime = 0

        self.shuttingDown = False

        self.t = threading.Thread(target=self.run)
        self.handlerFunctionMutex = threading.Lock()
        self.sendCommandHandlerFunction = lambda cycleTime, robot : cycleTime
        self.readResultHandlerFunction = lambda cycleTime, robot : cycleTime

        self.dcServoArray[0].setOffsetAndScaling(2 * pi / 4096.0, 0.950301, 0)
        self.dcServoArray[1].setOffsetAndScaling(2 * pi / 4096.0, -2.042107923, pi / 2)
        self.dcServoArray[2].setOffsetAndScaling(2 * pi / 4096.0, 1.0339, pi / 2)
        self.dcServoArray[3].setOffsetAndScaling(-2 * pi / 4096.0, -1.47531, 0.0)
        self.dcServoArray[4].setOffsetAndScaling(2 * pi / 4096.0, 1.23394, 0.0)
        self.dcServoArray[5].setOffsetAndScaling(-2 * pi / 4096.0, -1.45191, 0.0)

        self.dcServoArray[0].setControlSpeed(20)
        self.dcServoArray[0].setBacklashControlSpeed(0, 3.0, 0.00)
        self.dcServoArray[0].setFrictionCompensation(0)
        self.dcServoArray[1].setControlSpeed(20)
        self.dcServoArray[1].setBacklashControlSpeed(0, 3.0, 0.00)
        self.dcServoArray[1].setFrictionCompensation(0)
        self.dcServoArray[2].setControlSpeed(20)
        self.dcServoArray[2].setBacklashControlSpeed(0, 3.0, 0.00)
        self.dcServoArray[2].setFrictionCompensation(0)
        self.dcServoArray[3].setControlSpeed(20)
        self.dcServoArray[3].setBacklashControlSpeed(0, 3.0, 0.0)
        self.dcServoArray[3].setFrictionCompensation(0)
        self.dcServoArray[4].setControlSpeed(20)
        self.dcServoArray[4].setBacklashControlSpeed(0, 3.0, 0.0)
        self.dcServoArray[4].setFrictionCompensation(0)
        self.dcServoArray[5].setControlSpeed(20)
        self.dcServoArray[5].setBacklashControlSpeed(0, 3.0, 0.0)
        self.dcServoArray[5].setFrictionCompensation(0)

        self.gripperServo.setOffsetAndScaling(pi / 1900.0, 0.0, 0.0);

        initFunction(self)

        while True:
            allDone = True
            for s in self.dcServoArray:
                allDone = allDone and s.isInitComplete()
                s.run()

            allDone = allDone and self.gripperServo.isInitComplete()
            self.gripperServo.run();

            if allDone:
                break

        self.currentPosition = []
        for s in self.dcServoArray:
            self.currentPosition.append(s.getPosition())

        for i in range(0, 2):
            self.gripperServo.setReference(pi / 2.0, 0.0, 0.0)
            self.gripperServo.run()
            self.gripperServo.getPosition()

        self.t.start()

    def run(self):
        sleepUntilTimePoint = time.time();

        while not self.shuttingDown:
            self.cycleSleepTime = sleepUntilTimePoint - time.time()
            time.sleep(max(0, sleepUntilTimePoint - time.time()))
            sleepUntilTimePoint += self.cycleTime

            with self.handlerFunctionMutex:
                tempSendHandlerFunction = self.sendCommandHandlerFunction
                tempReadHandlerFunction = self.readResultHandlerFunction

            tempSendHandlerFunction(self.cycleTime, self)

            for s in self.dcServoArray:
                s.run()

            self.gripperServo.run()

            for i, s in enumerate(self.dcServoArray):
                self.currentPosition[i] = s.getPosition()

            self.gripperServo.getPosition()
 
            tempReadHandlerFunction(self.cycleTime, self)

    def getPosition(self):
        return self.currentPosition

    def setHandlerFunctions(self, newSendCommandHandlerFunction, newReadResultHandlerFunction):
        with self.handlerFunctionMutex:
            self.sendCommandHandlerFunction = newSendCommandHandlerFunction
            self.readResultHandlerFunction = newReadResultHandlerFunction


    def removeHandlerFunctions(self):
        self.setHandlerFunctions(lambda cycleTime, robot : cycleTime, lambda cycleTime, robot : cycleTime)

    def shutdown(self):
        if not self.shuttingDown:
            self.shuttingDown = True
            self.t.join()

    def getCycleSleepTime(self):
        return self.cycleSleepTime

def loadtxtfile(file, cols):
    out = np.loadtxt((x.replace(':',',') for x in file), delimiter = ',', usecols = cols)
    return out

def shrinkArray(a, size):
    newA = []

    indexScale = len(a) / size

    oldIndex = 0

    while oldIndex < len(a):
        v = 0
        n = 0

        nextOldIndex = oldIndex + indexScale
        if nextOldIndex > len(a):
            nextOldIndex = len(a)

        for i in range(int(oldIndex), int(nextOldIndex)):
            v += a[i]
            n += 1

        v = v / n

        newA.append(v)
        oldIndex += indexScale

    return newA

def intArrayToString(a):
    string = "{"
    first = True
    for v in a:
        if not first:
            string += ", "
        first = False
        string += str(int(round(v)))
    string += "};"
    return string

def printAsEigenInit(mat, indent = ""):
    string = ""
    firstI = True
    for i in mat:
        if firstI:
            firstI = False
        else:
            string += ",\n" + indent
        firstJ = True
        for j in i:
            if firstJ:
                firstJ = False
            else:
                string += ", "
            string += str(j)
    string += ";\n"
    return string

@numba.jit(nopython=True)
def calcCovWithEndOfVectors(aVec, bVec, ca, cb, noiseDepresMemLenght,  dir):
    cov = 0

    aDiff = (aVec[-1] - aVec[-noiseDepresMemLenght]) / noiseDepresMemLenght
    bDiff = (bVec[-1] - bVec[-noiseDepresMemLenght]) / noiseDepresMemLenght

    if dir == False:
        aDiff = 0
        bDiff = 0

    for i, (a, b) in enumerate(zip(aVec[-noiseDepresMemLenght:], bVec[-noiseDepresMemLenght:])):
        a += aDiff * (noiseDepresMemLenght - i - 1)
        b += bDiff * (noiseDepresMemLenght - i - 1)

        cov += (ca - a)**2 + (cb - b)**2
    if dir == True:
        cov += noiseDepresMemLenght * (
            (ca - aVec[-noiseDepresMemLenght+1] -
                (aVec[-1] - aVec[-noiseDepresMemLenght]))**2 +
            (cb - bVec[-noiseDepresMemLenght+1] -
                (bVec[-1] - bVec[-noiseDepresMemLenght]))**2)
        
    return cov

@numba.jit(nopython=True)
def findBestFitt(data, modData, aVec, bVec, noiseDepresMemLenght):
    minCov = 1000000
    minIndex = 0
    done = False
    wrapIndex = 0

    for i, d in enumerate(modData):
        cov = calcCovWithEndOfVectors(aVec, bVec, d[0], d[1], noiseDepresMemLenght, False) #len(aVec) < 40)

        if minCov > cov:
            minIndex = i
            minCov = cov

    if len(modData) < len(data) * 0.2:
        for i in range(noiseDepresMemLenght, int(len(aVec) / 2)):
            cov = calcCovWithEndOfVectors(aVec, bVec, aVec[i], bVec[i], noiseDepresMemLenght, False)

            if minCov > cov:
                wrapIndex = i
                done = True
                break

    return (minIndex, done, wrapIndex)

@numba.jit(nopython=True)
def findBestFitt2(a, b, aVec, bVec):
    minCov = 1000000
    minIndex = 0

    for i, d in enumerate(zip(aVec[1:], bVec[1:])):
        cov = (aVec[i] - a)**2 + (bVec[i] - b)**2 + (aVec[i + 1] - a)**2 + (bVec[i + 1] - b)**2

        if minCov > cov:
            minIndex = i
            minCov = cov

    return minIndex + 1

@numba.jit(nopython=True)
def findWorstFitt(aVec, bVec):
    maxCov = 0
    maxIndex = 0

    for i, d in enumerate(zip(aVec[1:], bVec[1:])):
        a = aVec[i + 1]
        b = bVec[i + 1]

        nextI = i + 2
        if nextI == len(aVec):
            nextI = -1

        cov = (aVec[i] - a)**2 + 0 * (bVec[i] - b)**2 + (aVec[nextI] - a)**2 + 0 * (bVec[nextI] - b)**2

        if maxCov < cov:
            maxIndex = i
            maxCov = cov

    return maxIndex + 1

class OpticalEncoderDataVectorGenerator:
    def __init__(self, data, segment = 3000,
            constVelIndex = 10000, noiseDepresMemLenght = 5,
            shouldAbort=None, updateProgress=None):

        self.data = data[:, 0:2]
        self.noiseDepresMemLenght = noiseDepresMemLenght
        self.constVelIndex = constVelIndex

        self.shouldAbort = shouldAbort
        self.updateProgress = updateProgress

        a0 = self.data[0, 0]
        b0 = self.data[0, 1]
        armed = False
        startOfFirstRotation = None
        endOfFirstRotation = None
        for i, d in enumerate(self.data[0:1000, 0:2]):
            c = (d[0] - a0)**2 + (d[1] -b0)**2

            if startOfFirstRotation == None:
                if c > 50000:
                    startOfFirstRotation = i
            elif endOfFirstRotation == None:
                if not armed:
                    if lastCost > c:
                        armed = True
                else:
                    if lastCost < c:
                        endOfFirstRotation = i

            lastCost = c

        startIndex = startOfFirstRotation
        firstRotationLenght = endOfFirstRotation - startOfFirstRotation
        dirNoiseDepresMemLenght = int(math.ceil(firstRotationLenght / 10))

        self.a0 = self.data[startIndex, 0]
        self.b0 = self.data[startIndex, 1]
        self.a1 = self.data[startIndex + dirNoiseDepresMemLenght, 0]
        self.b1 = self.data[startIndex + dirNoiseDepresMemLenght, 1]

        self.aVecList = []
        self.bVecList = []

        self.aVec = np.zeros(512)
        self.bVec = np.zeros(512)

        filteredData = []
        filterSeg = []
        for d in data:
            if len(filterSeg) < 10:
                filterSeg.append([d[0], d[1]])
            else:
                cov = 0
                s = filterSeg[0]
                for e in filterSeg[1:]:
                    cov += (e[0] - s[0])**2
                    cov += (e[1] - s[1])**2

                if cov > 10000:
                    for d in filterSeg:
                        filteredData.append(d)

                filterSeg = []

        nr = int((len(filteredData) - constVelIndex) / segment)
        self.data = np.array(filteredData[0: nr * segment + constVelIndex])

        actNr = 0
        for i in range(0, nr):
            if self.shouldAbort != None:
                if self.shouldAbort():
                    return

            try:
                aVecShrunk, bVecShrunk, aVec, bVec = self.genVec(constVelIndex + segment * i, constVelIndex + segment * (i + 1))
                self.aVec += aVecShrunk
                self.bVec += bVecShrunk

                self.aVecList.append(aVec)
                self.bVecList.append(bVec)

                actNr += 1
                time.sleep(0.1)
            except Exception as e:
                print(format(e))

        if actNr == 0:
            raise Exception('Not enough data')

        self.aVec = self.aVec / actNr
        self.bVec = self.bVec / actNr

    def genVec(self, beginIndex, endIndex):
        aVec = numba.typed.List()
        bVec = numba.typed.List()
        for i in range(0, self.noiseDepresMemLenght):
            aVec.append(self.a0)
            bVec.append(self.b0)

        data = self.data[beginIndex:endIndex]
        modData = data[:]
        wrapIndex = 0

        totalDataLength = len(self.data) - self.constVelIndex

        while len(modData) > 0:
            if self.shouldAbort != None:
                if self.shouldAbort():
                    return

            if self.updateProgress != None:
                fraction = (endIndex - len(modData) - self.constVelIndex) / totalDataLength
                self.updateProgress(fraction)
            
            minIndex, done, wrapIndex = findBestFitt(data, modData, aVec, bVec, self.noiseDepresMemLenght)

            if done:
                break

            aVec.append(modData[minIndex, 0])
            bVec.append(modData[minIndex, 1])

            modData = np.delete(modData, minIndex, 0)

        aVec = aVec[wrapIndex:]
        bVec = bVec[wrapIndex:]

        while len(modData) > 0:
            if self.shouldAbort != None:
                if self.shouldAbort():
                    return

            if self.updateProgress != None:
                fraction = (endIndex - len(modData) - self.constVelIndex) / totalDataLength
                self.updateProgress(fraction)

            minIndex = findBestFitt2(modData[0, 0], modData[0, 1], aVec, bVec)

            aVec.insert(minIndex, modData[0, 0])
            bVec.insert(minIndex, modData[0, 1])

            modData = np.delete(modData, 0, 0)

        dirNoiseDepresMemLenght = int(len(data) / 50)

        if abs(self.a1 - self.a0) < 2 and abs(self.b1 - self.b0) < 2:
            raise Exception('No movement detected')

        dirSign = 0
        if abs(self.a1 - self.a0) > abs(self.b1 - self.b0):
            dirSign = (aVec[dirNoiseDepresMemLenght] - aVec[0]) / (self.a1 - self.a0)
        else:
            dirSign = (bVec[dirNoiseDepresMemLenght] - bVec[0]) / (self.b1 - self.b0)

        if dirSign < 0:
            aVec = aVec[::-1]
            bVec = bVec[::-1]

        aVecShrunk = shrinkArray(aVec, 512)
        bVecShrunk = shrinkArray(bVec, 512)

        return (np.array(aVecShrunk), np.array(bVecShrunk), aVec, bVec)

    def plotGeneratedVectors(self, box, classString = ''):
        oldAVec = None
        oldBVec = None

        if classString != '':
            aVecPattern = re.compile(r'(.*createMainEncoderHandler\(\)\s*\{(.*\n)*?\s*std\s*::\s*array\s*<\s*uint16_t\s*,\s*512\s*>\s*aVec\s*=\s*)\{([\d\s,]*)\};')
            bVecPattern = re.compile(r'(.*createMainEncoderHandler\(\)\s*\{(.*\n)*?\s*std\s*::\s*array\s*<\s*uint16_t\s*,\s*512\s*>\s*bVec\s*=\s*)\{([\d\s,]*)\};')
            
            temp1 = aVecPattern.search(classString)
            temp2 = bVecPattern.search(classString)

            if temp1 != None and temp2 != None:
                oldAVecStr = temp1.group(3)
                oldBVecStr = temp2.group(3)
                oldAVecStr = '[' + oldAVecStr + ']'
                oldBVecStr = '[' + oldBVecStr + ']'

                oldAVec = eval(oldAVecStr)
                oldBVec = eval(oldBVecStr)

                if len(oldAVec) <= 1:
                    oldAVec = None
                if len(oldBVec) <= 1:
                    oldBVec = None

            def aligneTo(aVec, bVec, refAVec, refBVec):
                def calcCost(aVec, bVec, refAVec, refBVec):
                    cost = 0
                    for d in zip(aVec, bVec, refAVec, refBVec):
                        cost += (d[0] - d[2])**2 + (d[1] - d[3])**2
                    return cost

                bestFittShift = 0
                bestFittCost = float('inf')
                for shift in range(0, 512):
                    tempA = aVec[-shift:] + aVec[0:-shift]
                    tempB = bVec[-shift:] + bVec[0:-shift]

                    cost = calcCost(tempA, tempB, refAVec, refBVec)

                    if cost < bestFittCost:
                        bestFittCost = cost
                        bestFittShift = shift

                tempA = aVec[-bestFittShift:] + aVec[0:-bestFittShift]
                tempB = bVec[-bestFittShift:] + bVec[0:-bestFittShift]
                return tempA, tempB

        fig = Figure(figsize=(5, 4), dpi=100)
        ax = fig.add_subplot()

        for d in zip(self.aVecList, self.bVecList):
            x = np.arange(len(d[0])) * len(self.aVec) / len(d[0])
            ax.plot(x, d[0], ',')
            ax.plot(x, d[1], ',')

        if oldAVec != None and oldBVec != None:
            oldAVec, oldBVec = aligneTo(oldAVec, oldBVec, self.aVec, self.bVec)

            ax.plot(oldAVec, 'm-')
            ax.plot(oldBVec, 'c-')

        ax.plot(self.aVec, 'r-')
        ax.plot(self.bVec, 'g-')

        canvas = FigureCanvas(fig)
        canvas.set_size_request(600, 400)
        box.add(canvas)

        box.show_all()

    def writeVectorsToConfigClassString(self, configClassString):
        aVecPattern = re.compile(r'(.*createMainEncoderHandler\(\)\s*\{(.*\n)*?\s*std\s*::\s*array\s*<\s*uint16_t\s*,\s*512\s*>\s*aVec\s*=\s*)\{[\d\s,]*\};')
        bVecPattern = re.compile(r'(.*createMainEncoderHandler\(\)\s*\{(.*\n)*?\s*std\s*::\s*array\s*<\s*uint16_t\s*,\s*512\s*>\s*bVec\s*=\s*)\{[\d\s,]*\};')
        
        temp1 = aVecPattern.search(configClassString)
        temp2 = bVecPattern.search(configClassString)
        if temp1 != None and temp2 != None:
            configClassString = re.sub(aVecPattern, r'\1' + intArrayToString(self.aVec), configClassString)
            configClassString = re.sub(bVecPattern, r'\1' + intArrayToString(self.bVec), configClassString)

            return configClassString

        return ''

    def getGeneratedVectors(self):
        out = ''
        out += 'std::array<uint16_t, 512 > aVec = ' + intArrayToString(self.aVec) + '\n'
        out += 'std::array<uint16_t, 512 > bVec = ' + intArrayToString(self.bVec)

        return out
            

def sign(v):
    if v >= 0:
        return 1.0
    return -1.0

class SystemIdentificationObject:
    def __init__(self, data, a=None, b=None, c=None, f=None, dt=None):
        if len(data) == 0:
            self.dt = dt
            self.velData = np.array([[0], [0]])
            self.pwmData = np.array([[0], [0]])

            self.servoModelParameters = np.array([[a], [b], [f], [c]])
            self.identifyCurrentSystemModel()
            return

        self.dt = data[1, 0] - data[0, 0]

        derivativeTimeSteps = 2
        tempVelData = 0 * data[:, 1]
        for i, d in enumerate(zip(data[derivativeTimeSteps:,1], data[0:-derivativeTimeSteps,1])):
            tempVelData[i + 1] = 0.75 * tempVelData[i] + 0.25 * (d[0] - d[1]) / (derivativeTimeSteps * self.dt)

        def minDiff(vec, v):
            min = vec[0]
            for d in vec:
                if abs(v - min) > abs(v - d):
                    min = d
            return min

        lastVel = None
        for i, d in enumerate(tempVelData[1:-1]):
            i += 1
            if lastVel == None:
                lastVel = d
            if abs(d - lastVel) < 300000:
                data[i, 1] = d
                lastVel = d
            else:
                data[i, 1] = lastVel
                lastVel = minDiff(tempVelData[i - 5:i], lastVel)

        data = data[1:-1]

        velData = data[:, 1]
        pwmData = data[:, 2]

        velData = np.array(velData)
        velData.shape = (len(velData),1)
        pwmData = np.array(pwmData)
        pwmData.shape = (len(pwmData),1)

        velData -= np.mean(velData)

        self.velData = velData
        self.pwmData = pwmData

        self.identifyServoSystemModel()
        self.identifyCurrentSystemModel()

    def getServoSystemModelParameters(self, outputDt):
        contineusEqiv = -math.log(self.servoModelParameters[0]) / self.dt
        scaledPoleA = math.exp(outputDt * -contineusEqiv)
        scaledB = self.servoModelParameters[1, 0] / self.dt
        return np.array([scaledPoleA, scaledB])

    def identifyCurrentSystemModel(self):
        self.currentModelParams = np.array([1.0, self.servoModelParameters[3,0]])
        return self.currentModelParams

    def identifyServoSystemModel(self):
        covDef = False
        cov = None#np.matrix([[0.0, 0.0, 0.0, 0.0],
            #[0.0, 0.0, 0.0, 0.0],
            #[0.0, 0.0, 0.0, 0.0],
            #[0.0, 0.0, 0.0, 0.0]])
        covYDef = False
        covY = None#np.matrix([[0.0],[0.0],[0.0],[0.0]])

        velData = np.abs(self.velData)
        pwmData = np.abs(self.pwmData)

        for d in zip(velData[1+5:-5], velData[0+5:-1-5], pwmData[0+5:-1-5], pwmData[0:-1-5-5], pwmData[0+5+5:-1]):
            if d[4][0] == d[3][0]:
                phi = np.matrix([[d[1][0]], [d[2][0]], [1.0], [d[1][0] * d[2][0]]])
                y = d[0][0]

                temp = phi * np.transpose(phi)
                if covDef == False:
                    covDef = True
                    cov = temp
                else:
                    cov += temp
                if covYDef == False:
                    covYDef = True
                    covY = y * phi
                else:
                    covY += y * phi

        self.servoModelParameters = np.linalg.solve(cov, covY)
        self.servoModelParameters[2,0] = -self.servoModelParameters[2,0] / self.servoModelParameters[1,0]
        self.servoModelParameters[3,0] = self.servoModelParameters[3,0] / self.servoModelParameters[1,0]
        return self.servoModelParameters

    def plotServoSystemModel(self, box):
        simVel = 0 * self.velData
        lastSimVel = None
        for i, d in enumerate(zip(self.velData[1:], self.velData[0:-1], self.pwmData[0:-1])):
            if lastSimVel == None:
                lastSimVel = d[1][0]
            if True:
                pwm = d[2][0]
                friction = 0
                if pwm > 0:
                    friction = -self.servoModelParameters[2,0]
                elif pwm < 0:
                    friction = self.servoModelParameters[2,0]
                simVel[i] = (self.servoModelParameters[0] * lastSimVel +
                    self.servoModelParameters[1] * (pwm + friction + self.servoModelParameters[3] * lastSimVel * abs(pwm)))
            else:
                simVel[i] = d[0][0]

            lastSimVel = simVel[i]

        fig = Figure(figsize=(5, 4), dpi=100)
        ax = fig.add_subplot()

        t = np.arange(len(simVel)) * self.dt
        ax.plot(t, self.velData, 'b')
        ax.plot(t, simVel, 'k')

        canvas = FigureCanvas(fig)
        canvas.set_size_request(600, 400)
        box.add(canvas)

        box.show_all()

class KalmanFilter(object):
    """docstring for KalmanFilter"""
    def __init__(self, dt, A, B, C):
        super(KalmanFilter, self).__init__()

        Aex = np.hstack((A, B))
        Aex = np.vstack((Aex, np.array([[0, 0, 1]])))

        Bex = np.vstack((B, np.array([[0]])))
        Cex = np.hstack((C, np.array([[0]])))

        xhatex = np.zeros(np.shape(Bex))

        self.A = Aex
        self.AInv = np.linalg.inv(Aex)
        self.B = Bex
        self.C = Cex

        AT = np.transpose(Aex)
        CT = np.transpose(Cex)

        x = range(1 * 4 * 4, 101 * 4 * 4, 4 * 4)
        vec1 = []
        vec2 = []
        vec3 = []
        for v in x:
            kalmanFilterSpeed = v
            poles = np.exp(dt * np.array([-1.0, -0.98, -0.96]) * kalmanFilterSpeed)
            plaseResult = scipy.signal.place_poles(AT,  CT, poles)

            vec1.append(plaseResult.gain_matrix[0, 0])
            vec2.append(plaseResult.gain_matrix[0, 1])
            vec3.append(plaseResult.gain_matrix[0, 2])

        p1 = np.polyfit(x, vec1, 3)
        p2 = np.polyfit(x, vec2, 3)
        p3 = np.polyfit(x, vec3, 3)


        self.polyK = np.vstack((p1, p2, p3))

        speed = 20 * 4 * 4
        speed2 = speed * speed
        speed3 = speed * speed * speed

        K = [self.polyK[0, 0] * speed3 + self.polyK[0, 1] * speed2 + self.polyK[0, 2] * speed + self.polyK[0, 3],
            self.polyK[1, 0] * speed3 + self.polyK[1, 1] * speed2 + self.polyK[1, 2] * speed + self.polyK[1, 3],
            self.polyK[2, 0] * speed3 + self.polyK[2, 1] * speed2 + self.polyK[2, 2] * speed + self.polyK[2, 3]]

class ServoModel(object):
    """docstring for ServoModel"""
    def __init__(self, dt, systemModel):
            super(ServoModel, self).__init__()
            self.dt = dt
            self.systemModel = systemModel

            dtp = self.dt
            dt2p = self.dt**2
            temp = self.systemModel.getServoSystemModelParameters(self.dt)
            a = temp[0]
            b = temp[1]

            self.A = np.array([[1, dtp], [0, a]])
            self.B = np.array([[dt2p / 2], [dtp]]) * b
            self.C = np.array([[1, 0]])

            self.kalmanFilter = KalmanFilter(dt, self.A, self.B, self.C)

    def writeModelToConfigClassString(self, configClassString):
        pwmToStallCurrentPattern = re.compile(r'((\s*)constexpr\s+float\s+pwmToStallCurrent\s*)\{[\d.-e]*\};')
        backEmfCurrentPattern = re.compile(r'((\s*)constexpr\s+float\s+backEmfCurrent\s*)\{[^\}]*\};')
        controlParametersPattern = re.compile(r'((\s*)class\s+ControlParameters\s+:.*\n\2\{)(.*\n)*?\2\};')
        
        temp = True
        temp = pwmToStallCurrentPattern.search(configClassString) != None and temp
        temp = backEmfCurrentPattern.search(configClassString) != None and temp
        temp = controlParametersPattern.search(configClassString) != None and temp
        if temp:
            out = r'\1{' + str(self.systemModel.currentModelParams[0]) + r'};'
            configClassString = re.sub(pwmToStallCurrentPattern, out, configClassString)

            out = r'\1{' + str(self.systemModel.currentModelParams[1]) + r'};'
            configClassString = re.sub(backEmfCurrentPattern, out, configClassString)

            out = r'\1\n'
            out += r'\2  public:\n'
            out += r'\2    //kalman filter observer vector\n'
            out += r'\2    static Eigen::Matrix<float, 3, 4> getKVector()\n'
            out += r'\2    {\n'
            out += r'\2        Eigen::Matrix<float, 3, 4> K;\n'
            out += r'\2        K << ' + printAsEigenInit(self.kalmanFilter.polyK, r'\2            ')
            out += r'\n'
            out += r'\2        return K;\n'
            out += r'\2    }\n'
            out += r'\n'
            out += r'\2    //system model A matrix\n'
            out += r'\2    static Eigen::Matrix3f getAMatrix()\n'
            out += r'\2    {\n'
            out += r'\2        Eigen::Matrix3f A;\n'
            out += r'\2        A << ' + printAsEigenInit(self.kalmanFilter.A, r'\2            ')
            out += r'\n'
            out += r'\2        return A;\n'
            out += r'\2    }\n'
            out += r'\n'
            out += r'\2    //system model invers A matrix\n'
            out += r'\2    static Eigen::Matrix3f getAInvMatrix()\n'
            out += r'\2    {\n'
            out += r'\2        Eigen::Matrix3f AInv;\n'
            out += r'\2        AInv << ' + printAsEigenInit(self.kalmanFilter.AInv, r'\2            ')
            out += r'\n'
            out += r'\2        return AInv;\n'
            out += r'\2    }\n'
            out += r'\n'
            out += r'\2    //system model B matrix\n'
            out += r'\2    static Eigen::Vector3f getBVector()\n'
            out += r'\2    {\n'
            out += r'\2        Eigen::Vector3f B;\n'
            out += r'\2        B << ' + printAsEigenInit(self.kalmanFilter.B, r'\2            ')
            out += r'\n'
            out += r'\2        return B;\n'
            out += r'\2    }\n'
            out += r'\n'
            out += r'\2    //system model friction comp value\n'
            out += r'\2    static float getFrictionComp()\n'
            out += r'\2    {\n'
            out += r'\2        return ' + str(self.systemModel.servoModelParameters[2,0]) + ';\n'
            out += r'\2    }\n'
            out += r'\2};'
            configClassString = re.sub(controlParametersPattern, out, configClassString)

            return configClassString

        return ''

    def getGeneratedModel(self):
        out = ''
        out += '--------------------------------------------------------------------------------\n'
        out += '\n'
        out += '        constexpr float pwmToStallCurrent{' + str(self.systemModel.currentModelParams[0]) + '};\n'
        out += '        constexpr float backEmfCurrent{' + str(self.systemModel.currentModelParams[1]) + '};\n'
        out += '\n'
        out += '--------------------------------------------------------------------------------\n'
        out += '\n'
        out += '    class ControlParameters : public SetupConfigHolder::DefaultControlParameters\n'
        out += '    {\n'
        out += '      public:\n'
        out += '        //kalman filter observer vector\n'
        out += '        static Eigen::Matrix<float, 3, 4> getKVector()\n'
        out += '        {\n'
        out += '            Eigen::Matrix<float, 3, 4> K;\n'
        out += '            K << ' + printAsEigenInit(self.kalmanFilter.polyK , '                ')
        out += '\n'
        out += '            return K;\n'
        out += '        }\n'
        out += '\n'
        out += '        //system model A matrix\n'
        out += '        static Eigen::Matrix3f getAMatrix()\n'
        out += '        {\n'
        out += '            Eigen::Matrix3f A;\n'
        out += '            A << ' + printAsEigenInit(self.kalmanFilter.A, '                ')
        out += '\n'
        out += '            return A;\n'
        out += '        }\n'
        out += '\n'
        out += '        //system model invers A matrix\n'
        out += '        static Eigen::Matrix3f getAInvMatrix()\n'
        out += '        {\n'
        out += '            Eigen::Matrix3f AInv;\n'
        out += '            AInv << ' + printAsEigenInit(self.kalmanFilter.AInv, '                ')
        out += '\n'
        out += '            return AInv;\n'
        out += '        }\n'
        out += '\n'
        out += '        //system model B matrix\n'
        out += '        static Eigen::Vector3f getBVector()\n'
        out += '        {\n'
        out += '            Eigen::Vector3f B;\n'
        out += '            B << ' + printAsEigenInit(self.kalmanFilter.B, '                ')
        out += '\n'
        out += '            return B;\n'
        out += '        }\n'
        out += '\n'
        out += '        //system model friction comp value\n'
        out += '        static float getFrictionComp()\n'
        out += '        {\n'
        out += '            return ' + str(self.systemModel.servoModelParameters[2,0]) + ';\n'
        out += '        }\n'
        out += '    };\n'
        out += '\n'
        out += '--------------------------------------------------------------------------------\n'

        return out

class PwmNonlinearityIdentifier(object):
    def __init__(self, data):
        self.data = data

        self.lookUpStepSize = 4

        self.timeList = []
        pwmList = []
        self.ampList = []
        ampSum = 0
        ampSumNr = 0
        startOfPwmValue = 0
        lastPwm = None
        for d in zip(self.data[:, 0], self.data[:, 1], self.data[:, 2]):
            if not lastPwm == d[2]:
                if not lastPwm == None:
                    self.timeList.append(d[0])
                    pwmList.append(lastPwm)
                    self.ampList.append(math.sqrt(ampSum / ampSumNr))

                lastPwm = d[2]
                startOfPwmValue = d[0]
                ampSum = 0
                ampSumNr = 0

            if d[0] - startOfPwmValue > 2.0:
                ampSum += d[1]**2
                ampSumNr += 1

        self.timeList.append(self.data[-1, 0])
        pwmList.append(lastPwm)
        self.ampList.append(math.sqrt(ampSum / ampSumNr))

        i = len(pwmList) - 1
        
        t = (1023 - pwmList[i - 1]) / (pwmList[i] - pwmList[i - 1])

        pwmList.append((pwmList[i] - pwmList[i - 1]) * t + pwmList[i - 1])
        self.timeList.append((self.timeList[i] - self.timeList[i - 1]) * t + self.timeList[i - 1])
        self.ampList.append(((self.ampList[i] * pwmList[i] - self.ampList[i - 1] * pwmList[i - 1]) * t 
            + self.ampList[i - 1] * pwmList[i - 1]) / 1023)

        lastZero = 0
        self.compList = [0]
        self.compListPwm = [0]
        for i, d in enumerate(zip(pwmList, self.ampList)):
            comp = d[0] * d[1] / self.ampList[-1]
            self.compList.append(comp)
            self.compListPwm.append(d[0])
            if comp < 20:
                lastZero = i + 1

        firstNoneZeroVal = self.compList[lastZero + 1]
        firstNoneZeroValPwm = self.compListPwm[lastZero + 1]

        for i in range(0, lastZero + 1):
            pwm = self.compListPwm[i]
            newValue = pwm**2 / firstNoneZeroValPwm**2 * firstNoneZeroVal
            self.compList[i] = newValue

        self.pwmNonlinearityCompLookUp = []

        for pwm in range(0, 1024, self.lookUpStepSize):
            index = -1
            for d in self.compList[0:-1]:
                if d < pwm:
                    index += 1
                else:
                    break

            t = (pwm - self.compList[index]) / (self.compList[index + 1] - self.compList[index])
            self.pwmNonlinearityCompLookUp.append(self.compListPwm[index] + (self.compListPwm[index + 1] - self.compListPwm[index]) * t)

    def plotGeneratedVector(self, box):
        fig = Figure(figsize=(5, 4), dpi=100)
        ax = fig.add_subplot()

        ax.plot(range(0, 1024, self.lookUpStepSize), range(0, 1024, self.lookUpStepSize), 'k-')
        ax.plot(range(0, 1024, self.lookUpStepSize), self.pwmNonlinearityCompLookUp, 'g+-')
        ax.plot(self.compList, self.compListPwm, 'c.')

        canvas = FigureCanvas(fig)
        canvas.set_size_request(600, 400)
        box.add(canvas)

        box.show_all()


    def writeLinearizationFunctionToConfigClassString(self, configClassString):
        linearizeVecPattern = re.compile(r'((\s*).*createCurrentController\(\)\s*\{(.*\n)*?)(\s*)auto\s+pwmHighFrqCompFun\s+=\s+\[\]\(uint16_t\s+in\)(.*\n)*?\4\};((.*\n)*?\2\})')
        
        temp = linearizeVecPattern.search(configClassString)
        if temp != None:
            lookUpSize = len(self.pwmNonlinearityCompLookUp)
            out = r'\1'
            out += r'\4auto pwmHighFrqCompFun = [](uint16_t in)\n'
            out += r'\4{\n'
            out += r'\4    std::array<uint16_t, ' + str(lookUpSize) + '> linearizeVec = ' + intArrayToString(self.pwmNonlinearityCompLookUp) + '\n'
            out += r'\n'
            out += r'\4    float t = in * (' + str(lookUpSize - 1.0) + ' / 1023.0);\n'
            out += r'\4    size_t index = std::min(static_cast<size_t>(t), (size_t)' + str(lookUpSize - 2) + ');\n'
            out += r'\4    t -= index;\n'
            out += r'\4    const uint16_t& a = linearizeVec[index];\n'
            out += r'\4    const uint16_t& b = linearizeVec[index + 1];\n'
            out += r'\n'
            out += r'\4    return static_cast<uint16_t>((b - a) * t + a);\n'
            out += r'\4};\6'
            configClassString = re.sub(linearizeVecPattern, out, configClassString)

            return configClassString

        return ''

    def getLinearizationFunction(self):
        lookUpSize = len(self.pwmNonlinearityCompLookUp)
        out = ''
        out += 'auto pwmHighFrqCompFun = [](uint16_t in)\n'
        out += '{\n'
        out += '    std::array<uint16_t, ' + str(lookUpSize) + '> linearizeVec = ' + intArrayToString(self.pwmNonlinearityCompLookUp) + '\n'
        out += '\n'
        out += '    float t = in * (' + str(lookUpSize - 1.0) + ' / 1023.0);\n'
        out += '    size_t index = std::min(static_cast<size_t>(t), (size_t)' + str(lookUpSize - 2) + ');\n'
        out += '    t -= index;\n'
        out += '    const uint16_t& a = linearizeVec[index];\n'
        out += '    const uint16_t& b = linearizeVec[index + 1];\n'
        out += '\n'
        out += '    return static_cast<uint16_t>((b - a) * t + a);\n'
        out += '};'

        return out

class OutputEncoderCalibrationGenerator(object):
    def __init__(self, data, wrapAround, unitsPerRev):
        data[:, 1] = data[:, 1] * 4096 / unitsPerRev
        data[:, 2] = data[:, 2] * 4096 / unitsPerRev
        self.data = data

        minPos = min(self.data[:, 1])
        maxPos = max(self.data[:, 1])
        posListSize = int(4096 / 8)
        if not wrapAround:
            posListSize += 1

        posList = []
        for i in range(0, posListSize):
            posList.append([])

        for d in self.data:
            if d[1] != maxPos and d[1] != minPos:
                pos = int(round((d[1] % 4096) / 8))
                if pos == posListSize:
                    pos = posListSize - 1
                posList[pos].append(d[2])

        backlashSizeSum = 0
        backlashSizeNr = 0
        for d in posList:
            if len(d) >= 2:
                backlashSizeSum += max(d) - min(d)
                backlashSizeNr += 1

        meanBacklashSize = backlashSizeSum / backlashSizeNr

        self.minList = []
        self.maxList = []
        self.meanList = []
        for d in posList:
            temp = d
            if len(temp) == 0:
                self.minList.append(None)
                self.maxList.append(None)
                self.meanList.append(None)
            else:
                maxV = max(temp)
                minV = min(temp)
                meanV = (maxV + minV) / 2
                minSum = 0
                minNr = 0
                maxSum = 0
                maxNr = 0
                for d in temp:
                    if d < meanV - meanBacklashSize / 2 * 0.5:
                        minSum += d
                        minNr += 1
                    elif d > meanV + meanBacklashSize / 2 * 0.5:
                        maxSum += d
                        maxNr += 1
                
                if minNr != 0 and maxNr != 0:
                    self.minList.append(minSum / minNr)
                    self.maxList.append(maxSum / maxNr)
                    self.meanList.append((self.minList[-1] + self.maxList[-1]) / 2)
                else:
                    self.minList.append(None)
                    self.maxList.append(None)
                    self.meanList.append(None)

        i = 0
        if wrapAround:
            while self.meanList[i - 1] == None:
                i -= 1

        noneSegList = []
        end = i + len(self.meanList)
        while True:
            noneSeg = []
            while self.meanList[i] != None:
                i += 1
                if i == end:
                    break
            if i == end:
                break
            while self.meanList[i] == None:
                noneSeg.append(i)
                i += 1
                if i == end:
                    break
            noneSegList.append(noneSeg)
            if i == end:
                break

        for d1 in noneSegList:
            for d2 in d1:
                t = (d2 - (d1[0] - 1)) / ((d1[-1] + 1) - (d1[0] - 1))
                if not wrapAround and d1[0] - 1 < 0:
                    a = self.meanList[d1[-1] + 1]
                else:
                    a = self.meanList[d1[0] - 1]
                if not wrapAround and d1[-1] + 1 == len(self.meanList):
                    b = self.meanList[d1[0] - 1]
                else:
                    b = self.meanList[d1[-1] + 1]
                
                self.meanList[d2] = (b - a) * t + a

        if wrapAround:
            self.meanList.append(self.meanList[0])

        self.meanList = np.array(self.meanList)
        self.data[:, 2] -= np.mean(self.meanList)
        self.meanList -= np.mean(self.meanList)


    def plotGeneratedVector(self, box):
        fig = Figure(figsize=(5, 4), dpi=100)
        ax = fig.add_subplot()

        ax.plot(self.data[:, 1] % 4096, self.data[:, 2], 'b,')
        #ax.plot(self.minList, 'b-+')
        #ax.plot(self.maxList, 'r-+')
        ax.plot(range(0, 4096 + 8, 8), self.meanList, 'g-+')

        canvas = FigureCanvas(fig)
        canvas.set_size_request(600, 400)
        box.add(canvas)

        box.show_all()

    def writeVectorToConfigClassString(self, configClassString):
        compVecPattern = re.compile(r'(.*createOutputEncoderHandler\(\)\s*\{(.*\n)*?\s*std\s*::\s*array\s*<\s*int16_t\s*,\s*513\s*>\s*compVec\s*=\s*)\{\s*[^\}]*\s*\};')
        
        temp = compVecPattern.search(configClassString)
        if temp != None:
            configClassString = re.sub(compVecPattern, r'\1' + intArrayToString(self.meanList), configClassString)

            return configClassString

        return ''

    def getGeneratedVector(self):
        out = ''
        out += 'std::array<int16_t, 513 > compVec = ' + intArrayToString(self.meanList)

        return out


import os
import gi
import re
gi.require_version("Gtk", "3.0")
from gi.repository import GLib, Gtk

class GuiWindow(Gtk.Window):
    def __init__(self):
        Gtk.Window.__init__(self, title="Servo configuration", default_height=900, default_width=800)

        self.vboxMain = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        self.scrollWin = Gtk.ScrolledWindow()
        self.scrollWin.add(self.vboxMain)
        self.add(self.scrollWin)

        self.isClosed = False

        def closeEvent(widget, event):
            self.isClosed = True
        self.connect('delete-event', closeEvent)

        def transferToTargetMessage():
            dialog = Gtk.MessageDialog(
                    transient_for=self,
                    flags=0,
                    message_type=Gtk.MessageType.INFO,
                    buttons=Gtk.ButtonsType.OK,
                    text='Transfer to target needed!',
            )
            dialog.format_secondary_text(
                "Please transfer new configuration to target to apply changes"
            )
            response = dialog.run()
            dialog.destroy()

        def disconnectMotorFromGearboxMessage():
            dialog = Gtk.MessageDialog(
                    transient_for=self,
                    flags=0,
                    message_type=Gtk.MessageType.INFO,
                    buttons=Gtk.ButtonsType.OK,
                    text='Disconnect motor from gearbox!',
            )
            dialog.format_secondary_text(
                "Please make sure the motor can run freely by disconnecting it from the gearbox before continuing"
            )
            response = dialog.run()
            dialog.destroy()

        def startManuallyCalibrationMessage(timeString):
            dialog = Gtk.MessageDialog(
                    transient_for=self,
                    flags=0,
                    message_type=Gtk.MessageType.INFO,
                    buttons=Gtk.ButtonsType.OK,
                    text='Start manual calibration',
            )
            dialog.format_secondary_text(
                "Please connect the motor to the gearbox and move the servo-output-shaft "
                "back and forth for " + timeString + " over the whole range of motion, "
                "with constant speed."
            )
            response = dialog.run()
            dialog.destroy()

        def nullFunEvent(widget):
            pass

        def passOnScroll(widget, event):
                widget.stop_emission_by_name('scroll-event')

        def createLabel(text):
            label = Gtk.Label(label=text)
            label.set_use_markup(True)
            label.set_margin_start(30)
            label.set_margin_end(50)
            label.set_margin_top(8)
            label.set_margin_bottom(10)
            label.set_xalign(0.0)

            return label

        def addTopLabelTo(text, widget):
            boxHorizontal = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
            boxHorizontal.add(widget)

            label = Gtk.Label(label=text)
            label.set_use_markup(True)
            label.set_margin_start(30)
            label.set_margin_end(50)
            label.set_margin_top(8)
            label.set_margin_bottom(10)
            label.set_xalign(0.0)

            box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
            box.add(label)
            box.add(boxHorizontal)

            return box

        def creatComboBox(currentItem, itemList, onChangeFun = nullFunEvent, getLowLev = False):
            activeIndex = -1
            items = Gtk.ListStore(str)
            for i, name in enumerate(itemList):
                n = []
                n.append(name)
                items.append(n)

                if currentItem == name:
                    activeIndex = i

            comboBox = Gtk.ComboBox.new_with_model(items)
            comboBox.set_margin_start(40)
            comboBox.set_margin_end(10)
            comboBox.set_margin_top(10)
            comboBox.set_margin_bottom(8)
            renderer_text = Gtk.CellRendererText()
            comboBox.pack_start(renderer_text, True)
            comboBox.add_attribute(renderer_text, "text", 0)
            comboBox.set_active(activeIndex)
            comboBox.connect("changed", onChangeFun)
            comboBox.connect("scroll-event", passOnScroll)

            eventBox = Gtk.EventBox()
            box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
            box.add(comboBox)
            eventBox.add(box)

            if getLowLev:
                return eventBox, comboBox

            return eventBox

        def creatSpinButton(startValue, minValue, maxValue, stepSize, onChangeFun = nullFunEvent, getLowLev = False):
            spinButton = Gtk.SpinButton.new_with_range(min=minValue, max=maxValue, step=stepSize)
            spinButton.set_value(startValue)
            spinButton.set_margin_start(40)
            spinButton.set_margin_end(50)
            spinButton.set_margin_top(10)
            spinButton.set_margin_bottom(8)
            spinButton.connect("changed", onChangeFun)
            spinButton.connect("scroll-event", passOnScroll)

            box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
            box.add(spinButton)

            if getLowLev:
                return box, spinButton

            return box

        def createButton(text, onClickFun = nullFunEvent, width = 500, getLowLev = False):
            button = Gtk.Button(label=text)
            button.connect("clicked", onClickFun)
            button.set_margin_start(40)
            button.set_margin_end(50)
            button.set_margin_top(8)
            button.set_margin_bottom(10)
            button.set_property("width-request", width)

            box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
            box.add(button)

            if getLowLev:
                return box, button

            return box

        def createEntry(initText, onEdit = nullFunEvent, width = 500, getLowLev = False):
            entry = Gtk.Entry()
            entry.set_text(initText)
            entry.connect('changed', onEdit)
            entry.set_margin_start(40)
            entry.set_margin_end(50)
            entry.set_margin_top(8)
            entry.set_margin_bottom(10)
            entry.set_property("width-request", width)

            box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
            box.add(entry)

            if getLowLev:
                return box, entry

            return box

        def createLabelBox(text, orientation=Gtk.Orientation.VERTICAL):

            label = Gtk.Label(label=text)
            label.set_use_markup(True)
            label.set_margin_start(10)
            label.set_margin_end(10)
            label.set_margin_top(8)
            label.set_margin_bottom(10)
            label.set_xalign(0.0)

            box = Gtk.Box(orientation=orientation)

            box.add(label)

            return box

        def creatHScale(startValue, minValue, maxValue, stepSize, onChangeFun = nullFunEvent, width = 500, getLowLev = False):
            scale = Gtk.Scale.new_with_range(orientation=Gtk.Orientation.HORIZONTAL, min=minValue, max=maxValue, step=stepSize)
            scale.set_value(startValue)
            scale.set_margin_start(40)
            scale.set_margin_end(50)
            scale.set_margin_top(10)
            scale.set_margin_bottom(8)
            scale.set_property("width-request", width)
            scale.connect("value-changed", onChangeFun)
            scale.connect("scroll-event", passOnScroll)

            box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
            box.add(scale)

            if getLowLev:
                return box, scale

            return box

        def creatProgressBar(label, width = 500, getLowLev = False):
            progressBar = Gtk.ProgressBar()
            if label != '':
                progressBar.set_text(label)
                progressBar.set_show_text(True)
            progressBar.set_fraction(0.0)
            progressBar.set_margin_start(40)
            progressBar.set_margin_end(50)
            progressBar.set_margin_top(10)
            progressBar.set_margin_bottom(8)
            progressBar.set_property("width-request", width)

            box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
            box.add(progressBar)

            if getLowLev:
                return box, progressBar

            return box


        box = createLabelBox('<big><b>Create base config file</b></big>')
        baseConfigParamsBox = None

        def addButton(widget):
            nonlocal box
            nonlocal baseConfigParamsBox

            activeIter = widget.get_active_iter()
            if activeIter is not None:
                model = widget.get_model()
                configType = model[activeIter][0]

                if not baseConfigParamsBox == None:
                    box.remove(baseConfigParamsBox)

                baseConfigParamsBox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
                baseConfigParamsBox.set_margin_start(40)
                baseConfigParamsBox.set_margin_end(50)
                baseConfigParamsBox.set_margin_top(8)
                baseConfigParamsBox.set_margin_bottom(10)

                buttonCreateConfig = createButton('Create', getLowLev=True)

                def createConfigNameEntry(defaultValue):
                    nonlocal buttonCreateConfig

                    def checkOverrideExistingConfig(widget):
                        nonlocal buttonCreateConfig

                        configFileName = widget.get_text()

                        configs = self.getConfigurations()

                        for c in configs:
                            if c == configFileName:
                                buttonCreateConfig[1].set_label('Overwrite existing')
                                return

                        buttonCreateConfig[1].set_label('Create')

                    configNameEntry = createEntry(defaultValue, width = 250, onEdit=checkOverrideExistingConfig, getLowLev=True)
                    configNameEntry = addTopLabelTo('<b>Configuration file name (*.h)</b>\n for example: servoNr1.h', configNameEntry[0]), configNameEntry[1]
                    return configNameEntry

                def createNodeNrSpin(defaultValue):
                    nodeNrSpin = creatSpinButton(defaultValue, 1, 254, 1, getLowLev=True)
                    nodeNrSpin = addTopLabelTo('<b>Select servo nodeNr</b>',
                                nodeNrSpin[0]), nodeNrSpin[1]
                    return nodeNrSpin

                def createGearRatioEntry(defaultValue):
                    gearRatioEntry = createEntry(defaultValue, width = 350, getLowLev=True)
                    gearRatioEntry = addTopLabelTo(
                            '<b>Enter gear ratio from main encoder to output</b>\n must be a valid mathematical expression',
                            gearRatioEntry[0]), gearRatioEntry[1]
                    return gearRatioEntry

                def createOpticalEncoderPinCombos(defaultValuePin1, defaultValuePin2):
                    opticalEncoderPinBox = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
                    temp = Gtk.Label(label='Channel A:')
                    temp.set_margin_start(40)
                    opticalEncoderPinBox.pack_start(temp, False, False, 0)
                    pin1Combo = creatComboBox(defaultValuePin1, 
                                    ['A0',
                                    'A1',
                                    'A2',
                                    'A3',
                                    'A4',
                                    'A5'], getLowLev = True)
                    pin1Combo[1].set_margin_start(10)
                    pin1Combo[1].set_margin_end(10)
                    opticalEncoderPinBox.pack_start(pin1Combo[0], False, False, 0)
                    opticalEncoderPinBox.pack_start(Gtk.Label(label='Channel B:'), False, False, 0)
                    pin2Combo = creatComboBox(defaultValuePin2, 
                                    ['A0',
                                    'A1',
                                    'A2',
                                    'A3',
                                    'A4',
                                    'A5'], getLowLev = True)
                    pin2Combo[1].set_margin_start(10)
                    pwmInvertedCombo = pin2Combo[1]
                    opticalEncoderPinBox.pack_start(pin2Combo[0], False, False, 0)

                    opticalEncoderPinBox = addTopLabelTo('<b>Select analog pins for optical encoder</b>', opticalEncoderPinBox)
                    return opticalEncoderPinBox, pin1Combo[1], pin2Combo[1]

                def createMagEncoderCsPinComb(defaultValue):
                    encoderCSPinCombo = creatComboBox(defaultValue, 
                                    ['A0',
                                    'A1',
                                    'A2',
                                    'A3',
                                    'A4',
                                    'A5',
                                    '2',
                                    '3',
                                    '4',
                                    '7',
                                    '9',
                                    '10',
                                    '11',
                                    '12'], getLowLev=True)
                    encoderCSPinCombo = addTopLabelTo('<b>Select magnetic encoder chip select pin</b>',
                                encoderCSPinCombo[0]), encoderCSPinCombo[1]
                    return encoderCSPinCombo

                def createPotentiometerPinCombo(defaultPinValue, defaultRangeValue):
                    potentiometerConfBox = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
                    temp = Gtk.Label(label='Pin:')
                    temp.set_margin_start(40)
                    potentiometerConfBox.pack_start(temp, False, False, 0)
                    potentiometerPinCombo = creatComboBox(defaultPinValue,
                                    ['A0',
                                    'A1',
                                    'A2',
                                    'A3',
                                    'A4',
                                    'A5'], getLowLev=True)
                    potentiometerPinCombo[1].set_margin_start(10)
                    potentiometerPinCombo[1].set_margin_end(10)
                    potentiometerConfBox.pack_start(potentiometerPinCombo[0], False, False, 0)
                    potentiometerConfBox.pack_start(Gtk.Label(label='Range of motion in degrees:'), False, False, 0)
                    potentiometerRangeSpin = creatSpinButton(defaultRangeValue, 1, 360, 1, getLowLev=True)
                    potentiometerRangeSpin[1].set_margin_start(10)
                    pwmInvertedCombo = potentiometerRangeSpin[1]
                    potentiometerConfBox.pack_start(potentiometerRangeSpin[0], False, False, 0)

                    potentiometerConfBox = addTopLabelTo('<b>Potentiometer/resistive encoder configuration</b>', potentiometerConfBox)
                    return potentiometerConfBox, potentiometerPinCombo[1], potentiometerRangeSpin[1]


                def createPwmTypeCombos(defaultValue, defaultValueForInverted):
                    pwmConfigBox = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
                    pwmTypeCombo = creatComboBox(defaultValue, 
                                    ['20kHz on pin 11 and 12',
                                    '20kHz on pin 3 and 4',
                                    '730Hz on pin 11 and 12',
                                    '730Hz on pin 3 and 4'], getLowLev=True)
                    pwmTypeCombo[1].set_margin_end(10)
                    pwmConfigBox.pack_start(pwmTypeCombo[0], False, False, 0)
                    pwmConfigBox.pack_start(Gtk.Label(label='Inverted:'), False, False, 0)
                    pwmInvertedCombo = creatComboBox(defaultValueForInverted, 
                                    ['False',
                                    'True'], getLowLev=True)
                    pwmInvertedCombo[1].set_margin_start(10)
                    pwmConfigBox.pack_start(pwmInvertedCombo[0], False, False, 0)

                    pwmConfigBox = addTopLabelTo('<b>Select pwm config</b>\n 20kHz is better if motor allow it', pwmConfigBox)
                    return pwmConfigBox, pwmTypeCombo[1], pwmInvertedCombo[1]



                if configType == 'Servo with magnetic Encoder':
########################## Servo with magnetic Encoder ####################################################
                    configNameEntry = createConfigNameEntry('')
                    baseConfigParamsBox.pack_start(configNameEntry[0], False, False, 0)

                    nodeNrSpin = createNodeNrSpin(1)
                    baseConfigParamsBox.pack_start(nodeNrSpin[0], False, False, 0)

                    gearRatioEntry = createGearRatioEntry('275.0 / 125904.0')
                    baseConfigParamsBox.pack_start(gearRatioEntry[0], False, False, 0)
 
                    encoderCSPinCombo = createMagEncoderCsPinComb('A5')
                    baseConfigParamsBox.pack_start(encoderCSPinCombo[0], False, False, 0)

                    opticalEncoderPinCombos = createOpticalEncoderPinCombos('A2', 'A3')
                    baseConfigParamsBox.pack_start(opticalEncoderPinCombos[0], False, False, 0)

                    pwmTypeCombos = createPwmTypeCombos('20kHz on pin 11 and 12', 'False')
                    baseConfigParamsBox.pack_start(pwmTypeCombos[0], False, False, 0)

                    def createConfig(widget):
                        nonlocal buttonCreateConfig
                        nonlocal configNameEntry
                        nonlocal nodeNrSpin
                        nonlocal gearRatioEntry
                        nonlocal encoderCSPinCombo
                        nonlocal opticalEncoderPinCombos
                        nonlocal pwmTypeCombos

                        pwmTypes = ['HBridgeHighResPin11And12Pwm',
                                    'HBridgeHighResPin3And4Pwm']

                        parameterList = [configNameEntry[1].get_text(),
                                str(int(nodeNrSpin[1].get_value())),
                                gearRatioEntry[1].get_text(),
                                encoderCSPinCombo[1].get_model()[encoderCSPinCombo[1].get_active()][0],
                                opticalEncoderPinCombos[1].get_model()[opticalEncoderPinCombos[1].get_active()][0],
                                opticalEncoderPinCombos[2].get_model()[opticalEncoderPinCombos[2].get_active()][0],
                                pwmTypes[pwmTypeCombos[2].get_active()],
                                pwmTypeCombos[2].get_model()[pwmTypeCombos[2].get_active()][0]]
                        
                        configFileName = configNameEntry[1].get_text().strip()
                        if configFileName == '':
                            dialog = Gtk.MessageDialog(
                                    transient_for=self,
                                    flags=0,
                                    message_type=Gtk.MessageType.INFO,
                                    buttons=Gtk.ButtonsType.OK,
                                    text='No configuration file name given',
                            )
                            dialog.format_secondary_text(
                                "Please enter a valid configuration file name"
                            )
                            dialog.run()
                            dialog.destroy()
                            configNameEntry[1].grab_focus()
                            return
                        elif configFileName.find('.h') == 0:
                            dialog = Gtk.MessageDialog(
                                    transient_for=self,
                                    flags=0,
                                    message_type=Gtk.MessageType.INFO,
                                    buttons=Gtk.ButtonsType.OK,
                                    text='\".h\" is not a valid configuration file name',
                            )
                            dialog.format_secondary_text(
                                "Please enter a valid configuration file name"
                            )
                            dialog.run()
                            dialog.destroy()
                            configNameEntry[1].grab_focus()
                            return
                        elif not configFileName.find('.h') == len(configFileName) - 2:
                            dialog = Gtk.MessageDialog(
                                    transient_for=self,
                                    flags=0,
                                    message_type=Gtk.MessageType.INFO,
                                    buttons=Gtk.ButtonsType.OK,
                                    text='The configuration file name most end with \".h\"',
                            )
                            dialog.format_secondary_text(
                                "Please enter a valid configuration file name"
                            )
                            dialog.run()
                            dialog.destroy()
                            configNameEntry[1].grab_focus()
                            return

                        pinList = [encoderCSPinCombo[1].get_model()[encoderCSPinCombo[1].get_active()][0],
                                opticalEncoderPinCombos[1].get_model()[opticalEncoderPinCombos[1].get_active()][0],
                                opticalEncoderPinCombos[2].get_model()[opticalEncoderPinCombos[2].get_active()][0]]
                        if not pwmTypeCombos[1].get_model()[pwmTypeCombos[1].get_active()][0].find('11 and 12') == -1:
                            pinList.append('11')
                            pinList.append('12')
                        elif not pwmTypeCombos[1].get_model()[pwmTypeCombos[1].get_active()][0].find('3 and 4') == -1:
                            pinList.append('3')
                            pinList.append('4')
                        pinComboList = [encoderCSPinCombo[1], 
                                opticalEncoderPinCombos[1],
                                opticalEncoderPinCombos[2],
                                pwmTypeCombos[1],
                                pwmTypeCombos[1]]
                        for i, pin1 in enumerate(pinList):
                            for j, pin2 in enumerate(pinList[i + 1:]):
                                if pin1 == pin2:
                                    dialog = Gtk.MessageDialog(
                                            transient_for=self,
                                            flags=0,
                                            message_type=Gtk.MessageType.INFO,
                                            buttons=Gtk.ButtonsType.OK,
                                            text='The same pin cannot be used twice',
                                    )
                                    dialog.format_secondary_text(
                                        "Please change the pin"
                                    )
                                    dialog.run()
                                    dialog.destroy()
                                    pinComboList[i + 1 +j].grab_focus()
                                    return

                        out = ''
                        out += '#ifndef CONFIG_HOLDER_H\n'
                        out += '#define CONFIG_HOLDER_H\n'
                        out += '\n'
                        out += '#include "../defaultConfigHolder.h"\n'
                        out += '\n'
                        out += 'class SetupConfigHolder : public DefaultConfigHolder\n'
                        out += '{\n'
                        out += 'public:\n'
                        out += '    static std::unique_ptr<OpticalEncoderHandler> createMainEncoderHandler()\n'
                        out += '    {\n'
                        out += '        std::array<uint16_t, 512> aVec = {0};\n'
                        out += '        std::array<uint16_t, 512> bVec = {0};\n'
                        out += '        return std::make_unique<OpticalEncoderHandler>(aVec, bVec, '+ str(pinList[1]) + ', '+ str(pinList[2]) + ', 4096.0 * (' + gearRatioEntry[1].get_text() + '));\n'
                        out += '    }\n'
                        out += '\n'
                        out += '    static std::unique_ptr<EncoderHandlerInterface> createOutputEncoderHandler()\n'
                        out += '    {\n'
                        out += '        std::array<int16_t, 513> compVec = {0};\n'
                        out += '        return std::make_unique<EncoderHandler>(' + str(pinList[0]) + ', 4096.0, compVec);\n'
                        out += '    }\n'
                        out += '\n'
                        out += '    static std::unique_ptr<CurrentController> createCurrentController()\n'
                        out += '    {\n'
                        out += '        constexpr float pwmToStallCurrent{1.0};\n'
                        out += '        constexpr float backEmfCurrent{0.0};\n'
                        out += '\n'
                        out += '        auto pwmHighFrqCompFun = [](uint16_t in)\n'
                        out += '        {\n'
                        out += '            return in;\n'
                        out += '        };\n'
                        out += '\n'

                        pwmTypes = ['HBridgeHighResPin11And12Pwm',
                                    'HBridgeHighResPin3And4Pwm',
                                    'HBridge2WirePwm>(11, 12',
                                    'HBridge2WirePwm>(3, 4',
                                    'HBridge2WirePwm>(12, 11',
                                    'HBridge2WirePwm>(4, 3']

                        trueFalseList = ['false', 'true']

                        if pwmTypeCombos[1].get_active() == 0 or pwmTypeCombos[1].get_active() == 1:
                            out += '        return std::make_unique<CurrentControlModel>(pwmToStallCurrent, backEmfCurrent, std::make_unique<' + pwmTypes[pwmTypeCombos[1].get_active()] + '>(' + trueFalseList[pwmTypeCombos[2].get_active()] + ', pwmHighFrqCompFun));\n'

                        else:
                            out += '        return std::make_unique<CurrentControlModel>(pwmToStallCurrent, backEmfCurrent, std::make_unique<' + pwmTypes[pwmTypeCombos[1].get_active() + 2 * pwmTypeCombos[2].get_active()] + ', pwmHighFrqCompFun));\n'

                        out += '    }\n'
                        out += '\n'
                        out += '    class ControlParameters : public SetupConfigHolder::DefaultControlParameters\n'
                        out += '    {\n'
                        out += '      public:\n'
                        out += '        //kalman filter observer vector\n'
                        out += '        static Eigen::Vector3f getKVector()\n'
                        out += '        {\n'
                        out += '            Eigen::Vector3f K;\n'
                        out += '            K << 0.9408115641844931,\n'
                        out += '                232.9980363446573,\n'
                        out += '                21412.084555514386;\n'
                        out += '\n'
                        out += '            return K;\n'
                        out += '        }\n'
                        out += '\n'
                        out += '        //system model A matrix\n'
                        out += '        static Eigen::Matrix3f getAMatrix()\n'
                        out += '        {\n'
                        out += '            Eigen::Matrix3f A;\n'
                        out += '            A << 1.0, 0.0012, 7.199999999999999e-07,\n'
                        out += '                0.0, 1.0, 0.0012,\n'
                        out += '                0.0, 0.0, 1.0;\n'
                        out += '\n'
                        out += '            return A;\n'
                        out += '        }\n'
                        out += '\n'
                        out += '        //system model invers A matrix\n'
                        out += '        static Eigen::Matrix3f getAInvMatrix()\n'
                        out += '        {\n'
                        out += '            Eigen::Matrix3f AInv;\n'
                        out += '            AInv << 1.0, -0.0012, 7.199999999999999e-07,\n'
                        out += '                0.0, 1.0, -0.0012,\n'
                        out += '                0.0, 0.0, 1.0;\n'
                        out += '\n'
                        out += '            return AInv;\n'
                        out += '        }\n'
                        out += '\n'
                        out += '        //system model B matrix\n'
                        out += '        static Eigen::Vector3f getBVector()\n'
                        out += '        {\n'
                        out += '            Eigen::Vector3f B;\n'
                        out += '            B << 7.199999999999999e-07,\n'
                        out += '                0.0012,\n'
                        out += '                0.0;\n'
                        out += '\n'
                        out += '            return B;\n'
                        out += '        }\n'
                        out += '\n'
                        out += '        //system model friction comp value\n'
                        out += '        static float getFrictionComp()\n'
                        out += '        {\n'
                        out += '            return 0.0;\n'
                        out += '        }\n'
                        out += '    };\n'
                        out += '};\n'
                        out += '\n'
                        out += 'class ConfigHolder\n'
                        out += '{\n'
                        out += 'public:\n'
                        out += '    static std::unique_ptr<Communication> getCommunicationHandler()\n'
                        out += '    {\n'
                        out += '        Serial.begin(115200);\n'
                        out += '        Serial1.begin(115200);\n'
                        out += '        auto com = std::make_unique<Communication>(SerialComOptimizer(&Serial1, &Serial));\n'
                        out += '        com->addCommunicationNode(\n'
                        out += '                std::make_unique<DCServoCommunicationHandler>(' + str(int(nodeNrSpin[1].get_value())) + ', createDCServo<SetupConfigHolder>()));\n'
                        out += '\n'
                        out += '        return com;\n'
                        out += '    }\n'
                        out += '};\n'
                        out += '\n'
                        out += '#endif\n'

                        with open('config/' + configFileName, 'w') as configFile:
                            configFile.write(out)
                            if buttonCreateConfig[1].get_label() == 'Overwrite existing':
                                transferToTargetMessage()
                            buttonCreateConfig[1].set_label('Overwrite existing')

                    buttonCreateConfig[1].connect('clicked', createConfig)
                    buttonCreateConfig[0].set_margin_top(40)
                    baseConfigParamsBox.pack_start(buttonCreateConfig[0], False, False, 0)

                elif configType == 'Servo with potentiometer':
########################## Servo with potentiometer ####################################################
                    configNameEntry = createConfigNameEntry('')
                    baseConfigParamsBox.pack_start(configNameEntry[0], False, False, 0)

                    nodeNrSpin = createNodeNrSpin(1)
                    baseConfigParamsBox.pack_start(nodeNrSpin[0], False, False, 0)

                    gearRatioEntry = createGearRatioEntry('10.0 / 48.0 * 10.0 / 38.0 * 10.0 / 38.0 * 10.0 / 38.0')
                    baseConfigParamsBox.pack_start(gearRatioEntry[0], False, False, 0)
 
                    potentiometerPinCombo = createPotentiometerPinCombo('A0', 200)
                    baseConfigParamsBox.pack_start(potentiometerPinCombo[0], False, False, 0)

                    opticalEncoderPinCombos = createOpticalEncoderPinCombos('A1', 'A2')
                    baseConfigParamsBox.pack_start(opticalEncoderPinCombos[0], False, False, 0)

                    pwmTypeCombos = createPwmTypeCombos('20kHz on pin 11 and 12', 'True')
                    baseConfigParamsBox.pack_start(pwmTypeCombos[0], False, False, 0)

                    def createConfig(widget):
                        nonlocal buttonCreateConfig
                        nonlocal configNameEntry
                        nonlocal nodeNrSpin
                        nonlocal gearRatioEntry
                        nonlocal potentiometerPinCombo
                        nonlocal opticalEncoderPinCombos
                        nonlocal pwmTypeCombos

                        configFileName = configNameEntry[1].get_text().strip()
                        if configFileName == '':
                            dialog = Gtk.MessageDialog(
                                    transient_for=self,
                                    flags=0,
                                    message_type=Gtk.MessageType.INFO,
                                    buttons=Gtk.ButtonsType.OK,
                                    text='No configuration file name given',
                            )
                            dialog.format_secondary_text(
                                "Please enter a valid configuration file name"
                            )
                            dialog.run()
                            dialog.destroy()
                            configNameEntry[1].grab_focus()
                            return
                        elif configFileName.find('.h') == 0:
                            dialog = Gtk.MessageDialog(
                                    transient_for=self,
                                    flags=0,
                                    message_type=Gtk.MessageType.INFO,
                                    buttons=Gtk.ButtonsType.OK,
                                    text='\".h\" is not a valid configuration file name',
                            )
                            dialog.format_secondary_text(
                                "Please enter a valid configuration file name"
                            )
                            dialog.run()
                            dialog.destroy()
                            configNameEntry[1].grab_focus()
                            return
                        elif not configFileName.find('.h') == len(configFileName) - 2:
                            dialog = Gtk.MessageDialog(
                                    transient_for=self,
                                    flags=0,
                                    message_type=Gtk.MessageType.INFO,
                                    buttons=Gtk.ButtonsType.OK,
                                    text='The configuration file name most end with \".h\"',
                            )
                            dialog.format_secondary_text(
                                "Please enter a valid configuration file name"
                            )
                            dialog.run()
                            dialog.destroy()
                            configNameEntry[1].grab_focus()
                            return

                        pinList = [potentiometerPinCombo[1].get_model()[potentiometerPinCombo[1].get_active()][0],
                                opticalEncoderPinCombos[1].get_model()[opticalEncoderPinCombos[1].get_active()][0],
                                opticalEncoderPinCombos[2].get_model()[opticalEncoderPinCombos[2].get_active()][0]]
                        pinComboList = [potentiometerPinCombo[1], opticalEncoderPinCombos[1], opticalEncoderPinCombos[2]]
                        for i, pin1 in enumerate(pinList):
                            for j, pin2 in enumerate(pinList[i + 1:]):
                                if pin1 == pin2:
                                    dialog = Gtk.MessageDialog(
                                            transient_for=self,
                                            flags=0,
                                            message_type=Gtk.MessageType.INFO,
                                            buttons=Gtk.ButtonsType.OK,
                                            text='The same pin cannot be used twice',
                                    )
                                    dialog.format_secondary_text(
                                        "Please change the pin"
                                    )
                                    dialog.run()
                                    dialog.destroy()
                                    pinComboList[i + 1 +j].grab_focus()
                                    return

                        out = ''
                        out += '#ifndef CONFIG_HOLDER_H\n'
                        out += '#define CONFIG_HOLDER_H\n'
                        out += '\n'
                        out += '#include "../defaultConfigHolder.h"\n'
                        out += '\n'
                        out += 'class SetupConfigHolder : public DefaultConfigHolder\n'
                        out += '{\n'
                        out += 'public:\n'
                        out += '    static std::unique_ptr<OpticalEncoderHandler> createMainEncoderHandler()\n'
                        out += '    {\n'
                        out += '        std::array<uint16_t, 512> aVec = {0};\n'
                        out += '        std::array<uint16_t, 512> bVec = {0};\n'
                        out += '        return std::make_unique<OpticalEncoderHandler>(aVec, bVec, '+ str(pinList[1]) + ', '+ str(pinList[2]) + ', 4096.0 * (' + gearRatioEntry[1].get_text() + '));\n'
                        out += '    }\n'
                        out += '\n'
                        out += '    static std::unique_ptr<EncoderHandlerInterface> createOutputEncoderHandler()\n'
                        out += '    {\n'
                        out += '        std::array<int16_t, 513> compVec = {0};\n'
                        out += '        return std::make_unique<ResistiveEncoderHandler>(' + str(pinList[0]) + ', 4096.0 * ' + str(potentiometerPinCombo[2].get_value()) + ' / 360.0, compVec);\n'
                        out += '    }\n'
                        out += '\n'
                        out += '    static std::unique_ptr<CurrentController> createCurrentController()\n'
                        out += '    {\n'
                        out += '        constexpr float pwmToStallCurrent{1.0};\n'
                        out += '        constexpr float backEmfCurrent{0.0};\n'
                        out += '\n'
                        out += '        auto pwmHighFrqCompFun = [](uint16_t in)\n'
                        out += '        {\n'
                        out += '            return in;\n'
                        out += '        };\n'
                        out += '\n'

                        pwmTypes = ['HBridgeHighResPin11And12Pwm',
                                    'HBridgeHighResPin3And4Pwm',
                                    'HBridge2WirePwm>(11, 12',
                                    'HBridge2WirePwm>(3, 4',
                                    'HBridge2WirePwm>(12, 11',
                                    'HBridge2WirePwm>(4, 3']

                        trueFalseList = ['false', 'true']

                        if pwmTypeCombos[1].get_active() == 0 or pwmTypeCombos[1].get_active() == 1:
                            out += '        return std::make_unique<CurrentControlModel>(pwmToStallCurrent, backEmfCurrent, std::make_unique<' + pwmTypes[pwmTypeCombos[1].get_active()] + '>(' + trueFalseList[pwmTypeCombos[2].get_active()] + ', pwmHighFrqCompFun));\n'

                        else:
                            out += '        return std::make_unique<CurrentControlModel>(pwmToStallCurrent, backEmfCurrent, std::make_unique<' + pwmTypes[pwmTypeCombos[1].get_active() + 2 * pwmTypeCombos[2].get_active()] + ', pwmHighFrqCompFun));\n'

                        out += '    }\n'
                        out += '\n'
                        out += '    class ControlParameters : public SetupConfigHolder::DefaultControlParameters\n'
                        out += '    {\n'
                        out += '      public:\n'
                        out += '        //kalman filter observer vector\n'
                        out += '        static Eigen::Vector3f getKVector()\n'
                        out += '        {\n'
                        out += '            Eigen::Vector3f K;\n'
                        out += '            K << 1.586525387500536,\n'
                        out += '                318.7628570276654,\n'
                        out += '                25672.815087142895;\n'
                        out += '\n'
                        out += '            return K;\n'
                        out += '        }\n'
                        out += '\n'
                        out += '        //system model A matrix\n'
                        out += '        static Eigen::Matrix3f getAMatrix()\n'
                        out += '        {\n'
                        out += '            Eigen::Matrix3f A;\n'
                        out += '            A << 1.0, 0.0024, 2.8799999999999995e-06,\n'
                        out += '                0.0, 1.0, 0.0024,\n'
                        out += '                0.0, 0.0, 1.0;\n'
                        out += '\n'
                        out += '            return A;\n'
                        out += '        }\n'
                        out += '\n'
                        out += '        //system model invers A matrix\n'
                        out += '        static Eigen::Matrix3f getAInvMatrix()\n'
                        out += '        {\n'
                        out += '            Eigen::Matrix3f AInv;\n'
                        out += '            AInv << 1.0, -0.0024, 2.8799999999999995e-06,\n'
                        out += '                0.0, 1.0, -0.0024,\n'
                        out += '                0.0, 0.0, 1.0;\n'
                        out += '\n'
                        out += '            return AInv;\n'
                        out += '        }\n'
                        out += '\n'
                        out += '        //system model B matrix\n'
                        out += '        static Eigen::Vector3f getBVector()\n'
                        out += '        {\n'
                        out += '            Eigen::Vector3f B;\n'
                        out += '            B << 2.8799999999999995e-06,\n'
                        out += '                0.0024,\n'
                        out += '                0.0;\n'
                        out += '\n'
                        out += '            return B;\n'
                        out += '        }\n'
                        out += '\n'
                        out += '        //system model friction comp value\n'
                        out += '        static float getFrictionComp()\n'
                        out += '        {\n'
                        out += '            return 0.0;\n'
                        out += '        }\n'
                        out += '    };\n'
                        out += '};\n'
                        out += '\n'
                        out += 'class ConfigHolder\n'
                        out += '{\n'
                        out += 'public:\n'
                        out += '    static std::unique_ptr<Communication> getCommunicationHandler()\n'
                        out += '    {\n'
                        out += '        Serial.begin(115200);\n'
                        out += '        Serial1.begin(115200);\n'
                        out += '        auto com = std::make_unique<Communication>(SerialComOptimizer(&Serial1, &Serial));\n'
                        out += '        com->addCommunicationNode(\n'
                        out += '                std::make_unique<DCServoCommunicationHandler>(' + str(int(nodeNrSpin[1].get_value())) + ', createDCServo<SetupConfigHolder>()));\n'
                        out += '\n'
                        out += '        return com;\n'
                        out += '    }\n'
                        out += '};\n'
                        out += '\n'
                        out += '#endif\n'

                        with open('config/' + configFileName, 'w') as configFile:
                            configFile.write(out)
                            if buttonCreateConfig[1].get_label() == 'Overwrite existing':
                                transferToTargetMessage()
                            buttonCreateConfig[1].set_label('Overwrite existing')

                    buttonCreateConfig = [buttonCreateConfig[0],
                            buttonCreateConfig[1],
                            buttonCreateConfig[1].connect('clicked', createConfig)]

                    addSecServoButtonTuple = createButton('Add second servo config', width = 200, getLowLev = True)
                    addSecServoButtonTuple[0].set_margin_start(20)
                    addSecServoButtonTuple[0].set_margin_top(20)
                    addSecServoButtonTuple[0].set_margin_bottom(20)

                    def addSecServoConfig(widget):
########################## Second servo with potentiometer servo ####################################################
                        nonlocal buttonCreateConfig
                        nonlocal addSecServoButtonTuple

                        baseConfigParamsBox.remove(addSecServoButtonTuple[0])
                        baseConfigParamsBox.remove(buttonCreateConfig[0])

                        secServoConfigParamsBox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
                        secServoConfigParamsBox.set_margin_start(40)
                        secServoConfigParamsBox.set_margin_end(50)
                        secServoConfigParamsBox.set_margin_top(8)
                        secServoConfigParamsBox.set_margin_bottom(10)

                        secNodeNrSpin = createNodeNrSpin(1)
                        secServoConfigParamsBox.pack_start(secNodeNrSpin[0], False, False, 0)

                        secGearRatioEntry = createGearRatioEntry('10.0 / 48.0 * 10.0 / 38.0 * 10.0 / 38.0 * 10.0 / 38.0')
                        secServoConfigParamsBox.pack_start(secGearRatioEntry[0], False, False, 0)
     
                        secPotentiometerPinCombo = createPotentiometerPinCombo('A3', 200)
                        secServoConfigParamsBox.pack_start(secPotentiometerPinCombo[0], False, False, 0)

                        secOpticalEncoderPinCombos = createOpticalEncoderPinCombos('A4', 'A5')
                        secServoConfigParamsBox.pack_start(secOpticalEncoderPinCombos[0], False, False, 0)

                        secPwmTypeCombos = createPwmTypeCombos('20kHz on pin 3 and 4', 'True')
                        secServoConfigParamsBox.pack_start(secPwmTypeCombos[0], False, False, 0)

                        removeSecServoButtonTuple = createButton('Remove', width= 200, getLowLev = True)
                        #removeSecServoButtonTuple[1].set_margin_start(300)
                        secServoConfigParamsBox.pack_start(removeSecServoButtonTuple[0], False, False, 0)

                        secServoConfigParams = addTopLabelTo('<b>Second servo config</b>', 
                                secServoConfigParamsBox)
                        secServoConfigParams.set_margin_start(20)
                        secServoConfigParams.set_margin_top(20)
                        secServoConfigParams.set_margin_bottom(20)

                        def createDualConfig(widget):
                            nonlocal configNameEntry
                            nonlocal nodeNrSpin
                            nonlocal gearRatioEntry
                            nonlocal potentiometerPinCombo
                            nonlocal opticalEncoderPinCombos
                            nonlocal pwmTypeCombos

                            nonlocal secNodeNrSpin
                            nonlocal secGearRatioEntry
                            nonlocal secPotentiometerPinCombo
                            nonlocal secOpticalEncoderPinCombos
                            nonlocal secPwmTypeCombos
                            
                            configFileName = configNameEntry[1].get_text().strip()
                            if configFileName == '':
                                dialog = Gtk.MessageDialog(
                                        transient_for=self,
                                        flags=0,
                                        message_type=Gtk.MessageType.INFO,
                                        buttons=Gtk.ButtonsType.OK,
                                        text='No configuration file name given',
                                )
                                dialog.format_secondary_text(
                                    "Please enter a valid configuration file name"
                                )
                                dialog.run()
                                dialog.destroy()
                                configNameEntry[1].grab_focus()
                                return
                            elif configFileName.find('.h') == 0:
                                dialog = Gtk.MessageDialog(
                                        transient_for=self,
                                        flags=0,
                                        message_type=Gtk.MessageType.INFO,
                                        buttons=Gtk.ButtonsType.OK,
                                        text='\".h\" is not a valid configuration file name',
                                )
                                dialog.format_secondary_text(
                                    "Please enter a valid configuration file name"
                                )
                                dialog.run()
                                dialog.destroy()
                                configNameEntry[1].grab_focus()
                                return
                            elif not configFileName.find('.h') == len(configFileName) - 2:
                                dialog = Gtk.MessageDialog(
                                        transient_for=self,
                                        flags=0,
                                        message_type=Gtk.MessageType.INFO,
                                        buttons=Gtk.ButtonsType.OK,
                                        text='The configuration file name most end with \".h\"',
                                )
                                dialog.format_secondary_text(
                                    "Please enter a valid configuration file name"
                                )
                                dialog.run()
                                dialog.destroy()
                                configNameEntry[1].grab_focus()
                                return

                            if nodeNrSpin[1].get_value() == secNodeNrSpin[1].get_value():
                                dialog = Gtk.MessageDialog(
                                        transient_for=self,
                                        flags=0,
                                        message_type=Gtk.MessageType.INFO,
                                        buttons=Gtk.ButtonsType.OK,
                                        text='The same node number cannot be used twice',
                                )
                                dialog.format_secondary_text(
                                    "Please change the node number"
                                )
                                dialog.run()
                                dialog.destroy()
                                secNodeNrSpin[1].grab_focus()
                                return

                            pinList = [potentiometerPinCombo[1].get_model()[potentiometerPinCombo[1].get_active()][0],
                                    opticalEncoderPinCombos[1].get_model()[opticalEncoderPinCombos[1].get_active()][0],
                                    opticalEncoderPinCombos[2].get_model()[opticalEncoderPinCombos[2].get_active()][0],
                                    secPotentiometerPinCombo[1].get_model()[secPotentiometerPinCombo[1].get_active()][0],
                                    secOpticalEncoderPinCombos[1].get_model()[secOpticalEncoderPinCombos[1].get_active()][0],
                                    secOpticalEncoderPinCombos[2].get_model()[secOpticalEncoderPinCombos[2].get_active()][0]]
                            pinComboList = [potentiometerPinCombo[1], opticalEncoderPinCombos[1], opticalEncoderPinCombos[2],
                                    secPotentiometerPinCombo[1], secOpticalEncoderPinCombos[1], secOpticalEncoderPinCombos[2]]
                            for i, pin1 in enumerate(pinList):
                                for j, pin2 in enumerate(pinList[i + 1:]):
                                    if pin1 == pin2:
                                        dialog = Gtk.MessageDialog(
                                                transient_for=self,
                                                flags=0,
                                                message_type=Gtk.MessageType.INFO,
                                                buttons=Gtk.ButtonsType.OK,
                                                text='The same pin cannot be used twice',
                                        )
                                        dialog.format_secondary_text(
                                            "Please change the pin"
                                        )
                                        dialog.run()
                                        dialog.destroy()
                                        pinComboList[i + 1 +j].grab_focus()
                                        return

                            if ((pwmTypeCombos[1].get_model()[pwmTypeCombos[1].get_active()][0].find('11 and 12') != -1 and
                                        secPwmTypeCombos[1].get_model()[secPwmTypeCombos[1].get_active()][0].find('11 and 12') != -1) or
                                    (pwmTypeCombos[1].get_model()[pwmTypeCombos[1].get_active()][0].find('3 and 4') != -1 and
                                        secPwmTypeCombos[1].get_model()[secPwmTypeCombos[1].get_active()][0].find('3 and 4') != -1)):
                                dialog = Gtk.MessageDialog(
                                        transient_for=self,
                                        flags=0,
                                        message_type=Gtk.MessageType.INFO,
                                        buttons=Gtk.ButtonsType.OK,
                                        text='The same pin cannot be used twice',
                                )
                                dialog.format_secondary_text(
                                    "Please change the pin"
                                )
                                dialog.run()
                                dialog.destroy()
                                secPwmTypeCombos[1].grab_focus()
                                return

                            out = ''
                            out += '#ifndef CONFIG_HOLDER_H\n'
                            out += '#define CONFIG_HOLDER_H\n'
                            out += '\n'
                            out += '#include "../defaultConfigHolder.h"\n'
                            out += '\n'
                            out += 'class SetupConfigHolder : public DefaultConfigHolder\n'
                            out += '{\n'
                            out += 'public:\n'
                            out += '    static std::unique_ptr<OpticalEncoderHandler> createMainEncoderHandler()\n'
                            out += '    {\n'
                            out += '        std::array<uint16_t, 512> aVec = {0};\n'
                            out += '        std::array<uint16_t, 512> bVec = {0};\n'
                            out += '        return std::make_unique<OpticalEncoderHandler>(aVec, bVec, '+ str(pinList[1]) + ', '+ str(pinList[2]) + ', 4096.0 * (' + gearRatioEntry[1].get_text() + '));\n'
                            out += '    }\n'
                            out += '\n'
                            out += '    static std::unique_ptr<EncoderHandlerInterface> createOutputEncoderHandler()\n'
                            out += '    {\n'
                            out += '        std::array<int16_t, 513> compVec = {0};\n'
                            out += '        return std::make_unique<ResistiveEncoderHandler>(' + str(pinList[0]) + ', 4096.0 * ' + str(potentiometerPinCombo[2].get_value()) + ' / 360.0, compVec);\n'
                            out += '    }\n'
                            out += '\n'
                            out += '    static std::unique_ptr<CurrentController> createCurrentController()\n'
                            out += '    {\n'
                            out += '        constexpr float pwmToStallCurrent{1.0};\n'
                            out += '        constexpr float backEmfCurrent{0.0};\n'
                            out += '\n'
                            out += '        auto pwmHighFrqCompFun = [](uint16_t in)\n'
                            out += '        {\n'
                            out += '            return in;\n'
                            out += '        };\n'
                            out += '\n'

                            pwmTypes = ['HBridgeHighResPin11And12Pwm',
                                        'HBridgeHighResPin3And4Pwm',
                                        'HBridge2WirePwm>(11, 12',
                                        'HBridge2WirePwm>(3, 4',
                                        'HBridge2WirePwm>(12, 11',
                                        'HBridge2WirePwm>(4, 3']

                            trueFalseList = ['false', 'true']

                            if pwmTypeCombos[1].get_active() == 0 or pwmTypeCombos[1].get_active() == 1:
                                out += '        return std::make_unique<CurrentControlModel>(pwmToStallCurrent, backEmfCurrent, std::make_unique<' + pwmTypes[pwmTypeCombos[1].get_active()] + '>(' + trueFalseList[pwmTypeCombos[2].get_active()] + ', pwmHighFrqCompFun));\n'

                            else:
                                out += '        return std::make_unique<CurrentControlModel>(pwmToStallCurrent, backEmfCurrent, std::make_unique<' + pwmTypes[pwmTypeCombos[1].get_active() + 2 * pwmTypeCombos[2].get_active()] + ', pwmHighFrqCompFun));\n'

                            out += '    }\n'
                            out += '\n'
                            out += '    class ControlParameters : public SetupConfigHolder::DefaultControlParameters\n'
                            out += '    {\n'
                            out += '      public:\n'
                            out += '        //kalman filter observer vector\n'
                            out += '        static Eigen::Vector3f getKVector()\n'
                            out += '        {\n'
                            out += '            Eigen::Vector3f K;\n'
                            out += '            K << 1.586525387500536,\n'
                            out += '                318.7628570276654,\n'
                            out += '                25672.815087142895;\n'
                            out += '\n'
                            out += '            return K;\n'
                            out += '        }\n'
                            out += '\n'
                            out += '        //system model A matrix\n'
                            out += '        static Eigen::Matrix3f getAMatrix()\n'
                            out += '        {\n'
                            out += '            Eigen::Matrix3f A;\n'
                            out += '            A << 1.0, 0.0024, 2.8799999999999995e-06,\n'
                            out += '                0.0, 1.0, 0.0024,\n'
                            out += '                0.0, 0.0, 1.0;\n'
                            out += '\n'
                            out += '            return A;\n'
                            out += '        }\n'
                            out += '\n'
                            out += '        //system model invers A matrix\n'
                            out += '        static Eigen::Matrix3f getAInvMatrix()\n'
                            out += '        {\n'
                            out += '            Eigen::Matrix3f AInv;\n'
                            out += '            AInv << 1.0, -0.0024, 2.8799999999999995e-06,\n'
                            out += '                0.0, 1.0, -0.0024,\n'
                            out += '                0.0, 0.0, 1.0;\n'
                            out += '\n'
                            out += '            return AInv;\n'
                            out += '        }\n'
                            out += '\n'
                            out += '        //system model B matrix\n'
                            out += '        static Eigen::Vector3f getBVector()\n'
                            out += '        {\n'
                            out += '            Eigen::Vector3f B;\n'
                            out += '            B << 2.8799999999999995e-06,\n'
                            out += '                0.0024,\n'
                            out += '                0.0;\n'
                            out += '\n'
                            out += '            return B;\n'
                            out += '        }\n'
                            out += '\n'
                            out += '        //system model friction comp value\n'
                            out += '        static float getFrictionComp()\n'
                            out += '        {\n'
                            out += '            return 0.0;\n'
                            out += '        }\n'
                            out += '    };\n'
                            out += '};\n'


                            out += '\n'
                            out += 'class SetupConfigHolder2 : public DefaultConfigHolder\n'
                            out += '{\n'
                            out += 'public:\n'
                            out += '    static std::unique_ptr<OpticalEncoderHandler> createMainEncoderHandler()\n'
                            out += '    {\n'
                            out += '        std::array<uint16_t, 512> aVec = {0};\n'
                            out += '        std::array<uint16_t, 512> bVec = {0};\n'
                            out += '        return std::make_unique<OpticalEncoderHandler>(aVec, bVec, '+ str(pinList[4]) + ', '+ str(pinList[5]) + ', 4096.0 * (' + secGearRatioEntry[1].get_text() + '));\n'
                            out += '    }\n'
                            out += '\n'
                            out += '    static std::unique_ptr<EncoderHandlerInterface> createOutputEncoderHandler()\n'
                            out += '    {\n'
                            out += '        std::array<int16_t, 513> compVec = {0};\n'
                            out += '        return std::make_unique<ResistiveEncoderHandler>(' + str(pinList[3]) + ', 4096.0 * ' + str(secPotentiometerPinCombo[2].get_value()) + ' / 360.0, compVec);\n'
                            out += '    }\n'
                            out += '\n'
                            out += '    static std::unique_ptr<CurrentController> createCurrentController()\n'
                            out += '    {\n'
                            out += '        constexpr float pwmToStallCurrent{1.0};\n'
                            out += '        constexpr float backEmfCurrent{0.0};\n'
                            out += '\n'
                            out += '        auto pwmHighFrqCompFun = [](uint16_t in)\n'
                            out += '        {\n'
                            out += '            return in;\n'
                            out += '        };\n'
                            out += '\n'

                            if secPwmTypeCombos[1].get_active() == 0 or secPwmTypeCombos[1].get_active() == 1:
                                out += '        return std::make_unique<CurrentControlModel>(pwmToStallCurrent, backEmfCurrent, std::make_unique<' + pwmTypes[secPwmTypeCombos[1].get_active()] + '>(' + trueFalseList[secPwmTypeCombos[2].get_active()] + ', pwmHighFrqCompFun));\n'

                            else:
                                out += '        return std::make_unique<CurrentControlModel>(pwmToStallCurrent, backEmfCurrent, std::make_unique<' + pwmTypes[secPwmTypeCombos[1].get_active() + 2 * secPwmTypeCombos[2].get_active()] + ', pwmHighFrqCompFun));\n'

                            out += '    }\n'
                            out += '\n'
                            out += '    class ControlParameters : public SetupConfigHolder::DefaultControlParameters\n'
                            out += '    {\n'
                            out += '      public:\n'
                            out += '        //kalman filter observer vector\n'
                            out += '        static Eigen::Vector3f getKVector()\n'
                            out += '        {\n'
                            out += '            Eigen::Vector3f K;\n'
                            out += '            K << 1.586525387500536,\n'
                            out += '                318.7628570276654,\n'
                            out += '                25672.815087142895;\n'
                            out += '\n'
                            out += '            return K;\n'
                            out += '        }\n'
                            out += '\n'
                            out += '        //system model A matrix\n'
                            out += '        static Eigen::Matrix3f getAMatrix()\n'
                            out += '        {\n'
                            out += '            Eigen::Matrix3f A;\n'
                            out += '            A << 1.0, 0.0024, 2.8799999999999995e-06,\n'
                            out += '                0.0, 1.0, 0.0024,\n'
                            out += '                0.0, 0.0, 1.0;\n'
                            out += '\n'
                            out += '            return A;\n'
                            out += '        }\n'
                            out += '\n'
                            out += '        //system model invers A matrix\n'
                            out += '        static Eigen::Matrix3f getAInvMatrix()\n'
                            out += '        {\n'
                            out += '            Eigen::Matrix3f AInv;\n'
                            out += '            AInv << 1.0, -0.0024, 2.8799999999999995e-06,\n'
                            out += '                0.0, 1.0, -0.0024,\n'
                            out += '                0.0, 0.0, 1.0;\n'
                            out += '\n'
                            out += '            return AInv;\n'
                            out += '        }\n'
                            out += '\n'
                            out += '        //system model B matrix\n'
                            out += '        static Eigen::Vector3f getBVector()\n'
                            out += '        {\n'
                            out += '            Eigen::Vector3f B;\n'
                            out += '            B << 2.8799999999999995e-06,\n'
                            out += '                0.0024,\n'
                            out += '                0.0;\n'
                            out += '\n'
                            out += '            return B;\n'
                            out += '        }\n'
                            out += '\n'
                            out += '        //system model friction comp value\n'
                            out += '        static float getFrictionComp()\n'
                            out += '        {\n'
                            out += '            return 0.0;\n'
                            out += '        }\n'
                            out += '    };\n'
                            out += '};\n'


                            out += '\n'
                            out += 'class ConfigHolder\n'
                            out += '{\n'
                            out += 'public:\n'
                            out += '    static std::unique_ptr<Communication> getCommunicationHandler()\n'
                            out += '    {\n'
                            out += '        Serial.begin(115200);\n'
                            out += '        Serial1.begin(115200);\n'
                            out += '        auto com = std::make_unique<Communication>(SerialComOptimizer(&Serial1, &Serial));\n'
                            out += '        com->addCommunicationNode(\n'
                            out += '                std::make_unique<DCServoCommunicationHandler>(' + str(int(nodeNrSpin[1].get_value())) + ', createDCServo<SetupConfigHolder>()));\n'
                            out += '        com->addCommunicationNode(\n'
                            out += '                std::make_unique<DCServoCommunicationHandler>(' + str(int(secNodeNrSpin[1].get_value())) + ', createDCServo<SetupConfigHolder2>()));\n'
                            out += '\n'
                            out += '        return com;\n'
                            out += '    }\n'
                            out += '};\n'
                            out += '\n'
                            out += '#endif\n'

                            with open('config/' + configFileName, 'w') as configFile:
                                configFile.write(out)
                                if buttonCreateConfig[1].get_label() == 'Overwrite existing':
                                    transferToTargetMessage()
                                buttonCreateConfig[1].set_label('Overwrite existing')

                        buttonCreateConfig[1].disconnect(buttonCreateConfig[2])
                        buttonCreateConfig[2] = buttonCreateConfig[1].connect('clicked', createDualConfig)

                        def removeSecServoConfig(widget):
                            nonlocal createConfig
                            nonlocal buttonCreateConfig
                            nonlocal secServoConfigParams
                            nonlocal addSecServoButtonTuple

                            buttonCreateConfig[1].disconnect(buttonCreateConfig[2])
                            buttonCreateConfig[2] = buttonCreateConfig[1].connect('clicked', createConfig)

                            baseConfigParamsBox.remove(secServoConfigParams)
                            baseConfigParamsBox.remove(buttonCreateConfig[0])

                            baseConfigParamsBox.pack_start(addSecServoButtonTuple[0], False, False, 0)
                            baseConfigParamsBox.pack_start(buttonCreateConfig[0], False, False, 0)
                            baseConfigParamsBox.show_all()

                        removeSecServoButtonTuple[1].connect('clicked', removeSecServoConfig)

                        baseConfigParamsBox.pack_start(secServoConfigParams, False, False, 0)
                        baseConfigParamsBox.pack_start(buttonCreateConfig[0], False, False, 0)
                        baseConfigParamsBox.show_all()

                    addSecServoButtonTuple[1].connect('clicked', addSecServoConfig)
                    baseConfigParamsBox.pack_start(addSecServoButtonTuple[0], False, False, 0)

                    baseConfigParamsBox.pack_start(buttonCreateConfig[0], False, False, 0)

                box.add(baseConfigParamsBox)
                baseConfigParamsBox.show_all()

        box.pack_start(
                addTopLabelTo('<b>Select type</b>', creatComboBox('', 
                    ['',
                    'Servo with magnetic Encoder',
                    'Servo with potentiometer'], addButton)),
                False, False, 0)
        self.vboxMain.pack_start(box, False, False, 0)

        box0 = createLabelBox('<big><b>Active configuration</b></big>')
        self.vboxMain.pack_start(box0, False, False, 20)
        box1 = createLabelBox('<big><b>Calibration</b></big>')
        self.vboxMain.pack_start(box1, False, False, 20)

        activeConfigCombo = creatComboBox('', [''], getLowLev=True)
        def loadConfigs(widget, event):
            nonlocal activeConfigCombo
            configs = ['']
            for c in self.getConfigurations():
                configs.append(c)
            configs.sort()

            activeIndex = 0
            items = activeConfigCombo[1].get_model()
            currentItem = items[activeConfigCombo[1].get_active()][0]
            items = Gtk.ListStore(str)
            for i, name in enumerate(configs):
                n = []
                n.append(name)
                items.append(n)

                if currentItem == name:
                    activeIndex = i

            activeConfigCombo[1].set_model(items)
            activeConfigCombo[1].set_active(activeIndex)

        loadConfigs(None, None)
        activeConfigCombo[0].connect('enter-notify-event', loadConfigs)
        activeConfigCombo[1].connect('focus', loadConfigs)

        activeNodeNrCombo = creatComboBox('', [''], getLowLev=True)

        activeComPortCombo = creatComboBox('', [''], getLowLev=True)
        def loadComPorts(widget, event):
            nonlocal activeComPortCombo
            ports = []

            for port, desc, hwid in serial.tools.list_ports.comports():
                    ports.append(str(port) + ': ' + str(desc))
            ports.sort()
            ports.append(': Simulation')

            activeIndex = 0
            items = activeComPortCombo[1].get_model()
            currentItem = items[activeComPortCombo[1].get_active()][0]
            items = Gtk.ListStore(str)
            for i, name in enumerate(ports):
                n = []
                n.append(name)
                items.append(n)

                if currentItem == name:
                    activeIndex = i

            activeComPortCombo[1].set_model(items)
            activeComPortCombo[1].set_active(activeIndex)

        loadComPorts(None, None)
        activeComPortCombo[0].connect('enter-notify-event', loadComPorts)
        activeComPortCombo[1].connect('focus', loadComPorts)

        activeConfigCombo = addTopLabelTo('<b>Select active configuration for transfer</b>', activeConfigCombo[0]), activeConfigCombo[1]
        box0.pack_start(activeConfigCombo[0], False, False, 0)

        activeNodeNrCombo = addTopLabelTo('<b>Select node number</b>', activeNodeNrCombo[0]), activeNodeNrCombo[1]
        activeComPortCombo = addTopLabelTo('<b>Select COM port</b>', activeComPortCombo[0]), activeComPortCombo[1]
        box1.pack_start(activeComPortCombo[0], False, False, 0)

        lastActiveConfig = ''
        calibrationCombo = None
        calibrationBox = None
        def onActiveConfigChange(widget):
            nonlocal lastActiveConfig
            nonlocal calibrationCombo
            nonlocal activeNodeNrCombo
            nonlocal calibrationBox

            activeIter = widget.get_active_iter()
            if activeIter is not None:
                model = widget.get_model()
                configName = model[activeIter][0]


                if configName == lastActiveConfig:
                    return

                lastActiveConfig = configName

                if calibrationCombo != None:
                    box1.remove(calibrationCombo[0])
                    if calibrationBox != None:
                        box1.remove(calibrationBox)

                if configName == '':
                    items = Gtk.ListStore(str)
                    for i, name in enumerate(['']):
                        n = []
                        n.append(name)
                        items.append(n)

                    activeNodeNrCombo[1].set_model(items)
                    activeNodeNrCombo[1].set_active(0)
                    return

                with open("config/config.h", "w") as configFile:
                    configFile.write("#include \"" + configName + "\"\n")

                configFileAsString = ''
                with open("config/" + configName, "r") as configFile:
                    configFileAsString = configFile.read()

                    nodeNrList = []
                    configClassNames = []

                    tempConfigFileAsString = configFileAsString
                    dcServoPattern = re.compile(r'make_unique\s*<\s*DCServoCommunicationHandler\s*>\s*\(([0-9]*)\s*,\s*createDCServo\s*<\s*(\w+)\s*>')
                    temp = dcServoPattern.search(tempConfigFileAsString)
                    while temp != None:
                        nodeNrList.append(temp.group(1))
                        configClassNames.append(temp.group(2))
                        tempConfigFileAsString = tempConfigFileAsString[temp.end(1):]
                        temp = dcServoPattern.search(tempConfigFileAsString)

                    items = Gtk.ListStore(str)
                    for i, name in enumerate(nodeNrList):
                        n = []
                        n.append(name)
                        items.append(n)

                    activeNodeNrCombo[1].set_model(items)
                    activeNodeNrCombo[1].set_active(0)

                    if len(items) == 1:
                        box1.remove(activeNodeNrCombo[0])
                        box1.remove(activeComPortCombo[0])
                        box1.pack_start(activeComPortCombo[0], False, False, 0)
                    else:
                        box1.remove(activeNodeNrCombo[0])
                        box1.remove(activeComPortCombo[0])
                        box1.pack_start(activeNodeNrCombo[0], False, False, 0)
                        box1.pack_start(activeComPortCombo[0], False, False, 0)

                    supportedCalibrationOptions = ['',
                            'Optical encoder',
                            'Optical encoder (manual)',
                            'Pwm nonlinearity',
                            'System identification',
                            'Output encoder calibration',
                            'Test control loop']

                    def getNodeNrFromCombo(nodeNrCombo):
                        nodeNr = nodeNrCombo.get_model()[nodeNrCombo.get_active()][0]
                        nodeNr = int(nodeNr)

                        return nodeNr

                    def getComPortFromCombo(comPortCombo):
                        portString = comPortCombo.get_model()[comPortCombo.get_active()][0]
                        descStartIndex = portString.find(':')
                        if descStartIndex != -1:
                            return portString[0: descStartIndex]
                        return ''

                    def getConfigClassNameFromCombo(nodeNrCombo):
                        return configClassNames[nodeNrCombo.get_active()]

                    def createRobot(nodeNr, port, dt=0.004, initFunction=lambda a: a):
                        if port != '':
                            com = SerialCommunication(port)
                        else:
                            com = SimulateCommunication()
                        simVector = [True] * 7
                        simVector[nodeNr - 1] = False
                        robot = Robot(com, simVector, cycleTime=dt, initFunction=initFunction)

                        return robot

                    def onCaribrationTypeChange(widget):
                        nonlocal calibrationBox
                        nonlocal box1
                        calibrationType = ''
                        activeIter = widget.get_active_iter()
                        if activeIter is not None:
                            model = widget.get_model()
                            calibrationType = model[activeIter][0]

                        if calibrationBox != None:
                            box1.remove(calibrationBox)

                        if calibrationType == '':
                            calibrationBox = None

                        elif calibrationType == 'Optical encoder' or calibrationType == 'Optical encoder (manual)':
############################# Optical encoder #################################################################
                            manualMovement = False
                            if calibrationType == 'Optical encoder (manual)':
                                manualMovement = True
                            else:
                                disconnectMotorFromGearboxMessage()

                            calibrationBox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
                            calibrationBox.set_margin_start(40)
                            pwmValue = 0
                            if not manualMovement:
                                pwmValue = 1
                            pwmScale = creatHScale(pwmValue, 1, 1023, 10, getLowLev=True)
                            pwmScale = addTopLabelTo('<b>Motor pwm value</b>\n Choose a value that results in a moderate constant velocity', pwmScale[0]), pwmScale[1]
                            if not manualMovement:
                                calibrationBox.pack_start(pwmScale[0], False, False, 0)

                            testButton = createButton('Test pwm value', getLowLev=True)
                            startButton = createButton('Start calibration', getLowLev=True)

                            threadMutex = threading.Lock()
                            def updatePwmValue(widget):
                                nonlocal pwmValue
                                nonlocal threadMutex

                                with threadMutex:
                                    pwmValue = widget.get_value()

                            pwmScale[1].connect('value-changed', updatePwmValue)

                            def resetGuiAfterCalibration():
                                nonlocal startButton
                                nonlocal testButton
                                nonlocal calibrationBox
                                nonlocal recordingProgressBar
                                nonlocal analyzingProgressBar
                                nonlocal pwmScale

                                testButton[1].set_label('Test pwm value')
                                testButton[1].set_sensitive(True)
                                startButton[1].set_label('Start calibration')
                                startButton[1].set_sensitive(True)
                                calibrationBox.remove(recordingProgressBar[0])
                                calibrationBox.remove(analyzingProgressBar[0])
                                pwmScale[1].set_sensitive(True)

                            runThread = False
                            def testPwmRun(nodeNr, port):
                                nonlocal runThread
                                nonlocal pwmValue
                                nonlocal threadMutex
                                
                                try:
                                    robot = createRobot(nodeNr, port)

                                    t = 0.0
                                    doneRunning = False

                                    def sendCommandHandlerFunction(dt, robot):
                                        nonlocal nodeNr
                                        nonlocal t
                                        nonlocal pwmValue
                                        nonlocal threadMutex

                                        servo = robot.dcServoArray[nodeNr - 1]

                                        pwm = 0
                                        with threadMutex:
                                            pwm = pwmValue

                                        servo.setOpenLoopControlSignal(pwm, True)

                                    out = []

                                    def readResultHandlerFunction(dt, robot):
                                        nonlocal nodeNr
                                        nonlocal t
                                        nonlocal runThread
                                        nonlocal doneRunning

                                        t += dt
                                        servo = robot.dcServoArray[nodeNr - 1]
                                        opticalEncoderData = servo.getOpticalEncoderChannelData()
                                        out.append([t,
                                                opticalEncoderData.a,
                                                opticalEncoderData.b,
                                                opticalEncoderData.minCostIndex,
                                                opticalEncoderData.minCost,
                                                servo.getVelocity()])

                                        stop = False
                                        with threadMutex:
                                            if runThread == False:
                                                stop = True

                                        if stop:
                                            robot.removeHandlerFunctions()
                                            doneRunning = True

                                    robot.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction);

                                    while not doneRunning:
                                        time.sleep(0.1)

                                    robot.shutdown()

                                    data = np.array(out)

                                    def plotData(data):
                                        dialog = Gtk.MessageDialog(
                                                transient_for=self,
                                                flags=0,
                                                message_type=Gtk.MessageType.INFO,
                                                buttons=Gtk.ButtonsType.YES_NO,
                                                text='Pwm test done!',
                                        )
                                        dialog.format_secondary_text(
                                            "Do you want to plot the recorded data?"
                                        )
                                        response = dialog.run()
                                        dialog.destroy()

                                        if response == Gtk.ResponseType.YES:
                                            plt.figure(1)
                                            plt.plot(data[:, 0], data[:, 1], 'r')
                                            plt.plot(data[:, 0], data[:, 2], 'g')
                                            
                                            plt.figure(2)
                                            plt.plot(data[:, 0], data[:, 3])
                                            plt.figure(3)
                                            plt.plot(data[:, 0], data[:, 4])
                                            plt.figure(4)
                                            plt.plot(data[:, 3], data[:, 4], '+')
                                            plt.figure(5)
                                            plt.plot(data[:, 3], data[:, 5], '+')
                                            plt.show()

                                    GLib.idle_add(plotData, data)

                                except Exception as e:
                                    print(format(e))

                                GLib.idle_add(resetGuiAfterCalibration)

                            testThread = None

                            def onTestPwm(widget):
                                nonlocal calibrationBox
                                nonlocal startButton

                                nonlocal testThread
                                nonlocal runThread
                                nonlocal pwmValue
                                nonlocal threadMutex

                                if widget.get_label() == 'Test pwm value':
                                    startButton[1].set_sensitive(False)
                                    widget.set_label('Stop pwm test')
                                    with threadMutex:
                                        runThread = True
                                    nodeNr = getNodeNrFromCombo(activeNodeNrCombo[1])
                                    port = getComPortFromCombo(activeComPortCombo[1])
                                    testThread = threading.Thread(target=testPwmRun, args=(nodeNr, port,))
                                    testThread.start()
                                else:
                                    with threadMutex:
                                        runThread = False
                                    testThread.join()


                            testButton[1].connect('clicked', onTestPwm)
                            if not manualMovement:
                                calibrationBox.pack_start(testButton[0], False, False, 0)

                            recordingProgressBar = creatProgressBar(label='Recording', getLowLev=True)
                            analyzingProgressBar = creatProgressBar(label='Analyzing', getLowLev=True)

                            def updateRecordingProgressBar(fraction):
                                nonlocal recordingProgressBar

                                recordingProgressBar[1].set_fraction(fraction)

                            def updateAnalyzingProgressBar(fraction):
                                nonlocal analyzingProgressBar

                                analyzingProgressBar[1].set_fraction(fraction)

                            configClassName = getConfigClassNameFromCombo(activeNodeNrCombo[1])
                            
                            def startCalibrationRun(nodeNr, port):
                                nonlocal configClassName

                                nonlocal runThread
                                nonlocal pwmValue
                                nonlocal threadMutex

                                try:
                                    robot = createRobot(nodeNr, port)

                                    def handleResults(opticalEncoderDataVectorGenerator):
                                        nonlocal configName
                                        nonlocal configClassName

                                        classString = ''

                                        with open("config/" + configName, "r") as configFile:
                                            configFileAsString = configFile.read()

                                            classPattern =re.compile(r'(\s*class\s+' + configClassName + r'\s+(.*\n)*?(.*\n)*?\})')
                                            temp = classPattern.search(configFileAsString)
                                            if temp != None:
                                                classString = temp.group(0)

                                        dialog = Gtk.MessageDialog(
                                                transient_for=self,
                                                flags=0,
                                                message_type=Gtk.MessageType.INFO,
                                                buttons=Gtk.ButtonsType.YES_NO,
                                                text='Optical encoder calibration done!',
                                        )
                                        dialog.format_secondary_text(
                                            "Should the configuration be updated with the new data?"
                                        )
                                        opticalEncoderDataVectorGenerator.plotGeneratedVectors(dialog.get_message_area(), classString)
                                        response = dialog.run()
                                        dialog.destroy()

                                        if response == Gtk.ResponseType.YES:
                                            classString = opticalEncoderDataVectorGenerator.writeVectorsToConfigClassString(classString)
                                            if classString != '':
                                                configFileAsString = re.sub(classPattern, classString, configFileAsString)
                                                with open("config/" + configName, "w") as configFile:
                                                    configFile.write(configFileAsString)
                                                    transferToTargetMessage()

                                                    return

                                            dialog = Gtk.MessageDialog(
                                                    transient_for=self,
                                                    flags=0,
                                                    message_type=Gtk.MessageType.ERROR,
                                                    buttons=Gtk.ButtonsType.OK,
                                                    text='Configuration format error!',
                                            )
                                            dialog.format_secondary_text(
                                                "Please past in the new aVec and bVec manually"
                                            )
                                            box = dialog.get_message_area()
                                            vecEntry = Gtk.Entry()
                                            vecEntry.set_text(opticalEncoderDataVectorGenerator.getGeneratedVectors())
                                            box.add(vecEntry)
                                            box.show_all()
                                            response = dialog.run()
                                            dialog.destroy()

                                    def handleAnalyzeError(info):
                                        dialog = Gtk.MessageDialog(
                                                transient_for=self,
                                                flags=0,
                                                message_type=Gtk.MessageType.ERROR,
                                                buttons=Gtk.ButtonsType.OK,
                                                text='Calibration failed during analyzing',
                                        )
                                        dialog.format_secondary_text(info)
                                        response = dialog.run()
                                        dialog.destroy()


                                    t = 0.0
                                    doneRunning = False

                                    def sendCommandHandlerFunction(dt, robot):
                                        nonlocal nodeNr
                                        nonlocal t
                                        nonlocal pwmValue
                                        nonlocal threadMutex

                                        servo = robot.dcServoArray[nodeNr - 1]

                                        pwm = 0
                                        with threadMutex:
                                            pwm = pwmValue
                                        if t < 100.0:
                                            servo.setOpenLoopControlSignal(min(pwm, 0.25 * t * pwm), True)
                                        else:
                                            servo.setOpenLoopControlSignal(-pwm, True)


                                    out = []

                                    def readResultHandlerFunction(dt, robot):
                                        nonlocal nodeNr
                                        nonlocal t
                                        nonlocal runThread
                                        nonlocal doneRunning

                                        runTime = 210.0
                                        if manualMovement:
                                            runTime = 200.0

                                        servo = robot.dcServoArray[nodeNr - 1]
                                        opticalEncoderData = servo.getOpticalEncoderChannelData()
                                        if t > 0.1 and (t < 100.0 or t > 110.0 or manualMovement):
                                            out.append([t,
                                                    opticalEncoderData.a,
                                                    opticalEncoderData.b,
                                                    opticalEncoderData.minCostIndex,
                                                    opticalEncoderData.minCost])

                                        GLib.idle_add(updateRecordingProgressBar, t / runTime)

                                        stop = t >= runTime
                                        with threadMutex:
                                            if runThread == False:
                                                stop = True

                                        if stop:
                                            robot.removeHandlerFunctions()
                                            doneRunning = True

                                        t += dt

                                    robot.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction);

                                    while not doneRunning:
                                        time.sleep(0.1)

                                    robot.shutdown()

                                    data = np.array(out)

                                    def shouldAbort():
                                        nonlocal runThread
                                        nonlocal threadMutex

                                        with threadMutex:
                                            if runThread == False:
                                                return True
                                        return False

                                    lastFraction = 0.0
                                    def updateProgress(fraction):
                                        nonlocal lastFraction

                                        if abs(fraction - lastFraction) > 0.01:
                                            lastFraction = fraction
                                            GLib.idle_add(updateAnalyzingProgressBar, fraction)

                                    if not shouldAbort():
                                        try:
                                            opticalEncoderDataVectorGenerator = OpticalEncoderDataVectorGenerator(
                                                    data[:, 1:3], constVelIndex=1000, noiseDepresMemLenght=8,
                                                    shouldAbort=shouldAbort,
                                                    updateProgress=updateProgress)

                                            GLib.idle_add(handleResults, opticalEncoderDataVectorGenerator)
                                        except Exception as e:
                                            GLib.idle_add(handleAnalyzeError, format(e))

                                except Exception as e:
                                    print(format(e))

                                GLib.idle_add(resetGuiAfterCalibration)
                            
                            def onStartCalibration(widget):
                                nonlocal testThread
                                nonlocal threadMutex
                                nonlocal runThread

                                nonlocal calibrationBox
                                nonlocal pwmScale
                                nonlocal recordingProgressBar
                                nonlocal analyzingProgressBar
                                nonlocal testButton

                                if widget.get_label() == 'Start calibration':
                                    widget.set_label('Abort calibration')

                                    if manualMovement:
                                        startManuallyCalibrationMessage('200 seconds')

                                    recordingProgressBar[1].set_fraction(0.0)
                                    analyzingProgressBar[1].set_fraction(0.0)
                                    testButton[1].set_sensitive(False)
                                    calibrationBox.pack_start(recordingProgressBar[0], False, False, 0)
                                    calibrationBox.pack_start(analyzingProgressBar[0], False, False, 0)
                                    pwmScale[1].set_sensitive(False)

                                    calibrationBox.show_all()

                                    with threadMutex:
                                        runThread = True
                                    nodeNr = getNodeNrFromCombo(activeNodeNrCombo[1])
                                    port = getComPortFromCombo(activeComPortCombo[1])
                                    testThread = threading.Thread(target=startCalibrationRun, args=(nodeNr, port,))
                                    testThread.start()                                    
                                else:
                                    with threadMutex:
                                        runThread = False
                                    testThread.join()


                            startButton[1].connect('clicked', onStartCalibration)
                            calibrationBox.pack_start(startButton[0], False, False, 0)

                            calibrationBox.show_all()

                        elif calibrationType == 'Pwm nonlinearity':
############################# Pwm nonlinearity ################################################################
                            disconnectMotorFromGearboxMessage()

                            calibrationBox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
                            calibrationBox.set_margin_start(40)

                            maxOscillationFrq = 10 
                            maxFrqScale = creatHScale(maxOscillationFrq, 1, 100, 1, getLowLev=True)
                            maxFrqScale = addTopLabelTo('<b>Max oscillation frequency</b>', maxFrqScale[0]), maxFrqScale[1]
                            calibrationBox.pack_start(maxFrqScale[0], False, False, 0)

                            midPwmValue = 500
                            maxPwmValue = 605
                            midPwmScale = creatHScale(midPwmValue, 0, 1023, 10, getLowLev=True)
                            midPwmScale = addTopLabelTo('<b>Pwm value for switching to sparse sample points</b>\n Lower value limits motor heat up at the cost of calibration resolution', midPwmScale[0]), midPwmScale[1]
                            calibrationBox.pack_start(midPwmScale[0], False, False, 0)

                            maxPwmScale = creatHScale(maxPwmValue, 1, 1023, 10, getLowLev=True)
                            maxPwmScale = addTopLabelTo('<b>Max motor pwm value</b>', maxPwmScale[0]), maxPwmScale[1]
                            calibrationBox.pack_start(maxPwmScale[0], False, False, 0)

                            testButton = createButton('Test pwm value', getLowLev=True)
                            startButton = createButton('Start calibration', getLowLev=True)

                            recordingProgressBar = creatProgressBar(label='Recording', getLowLev=True)

                            testPwmValue = midPwmValue

                            threadMutex = threading.Lock()
                            def maxFrqValueChanged(widget):
                                nonlocal maxOscillationFrq
                                nonlocal threadMutex

                                with threadMutex:
                                    maxOscillationFrq = widget.get_value()

                            def midPwmValueChanged(widget):
                                nonlocal maxPwmScale
                                nonlocal midPwmValue
                                nonlocal testPwmValue
                                nonlocal threadMutex

                                with threadMutex:
                                    midPwmValue = widget.get_value()
                                    testPwmValue = midPwmValue

                                if midPwmValue > maxPwmScale[1].get_value():
                                    maxPwmScale[1].set_value(midPwmValue)

                            def maxPwmValueChanged(widget):
                                nonlocal midPwmScale
                                nonlocal maxPwmValue
                                nonlocal testPwmValue
                                nonlocal threadMutex

                                with threadMutex:
                                    maxPwmValue = widget.get_value()
                                    testPwmValue = maxPwmValue

                                if maxPwmValue < midPwmScale[1].get_value():
                                    midPwmScale[1].set_value(maxPwmValue)

                            maxFrqScale[1].connect('value-changed', maxFrqValueChanged)
                            midPwmScale[1].connect('value-changed', midPwmValueChanged)
                            maxPwmScale[1].connect('value-changed', maxPwmValueChanged)

                            def resetGuiAfterCalibration():
                                nonlocal startButton
                                nonlocal testButton
                                nonlocal calibrationBox
                                nonlocal recordingProgressBar
                                nonlocal maxFrqScale
                                nonlocal midPwmScale
                                nonlocal maxPwmScale

                                startButton[1].set_label('Start calibration')
                                startButton[1].set_sensitive(True)
                                testButton[1].set_label('Test pwm value')
                                testButton[1].set_sensitive(True)
                                calibrationBox.remove(recordingProgressBar[0])
                                maxFrqScale[1].set_sensitive(True)
                                midPwmScale[1].set_sensitive(True)
                                maxPwmScale[1].set_sensitive(True)

                            resetGuiAfterCalibration()

                            runThread = False
                            def testPwmRun(nodeNr, port):
                                nonlocal runThread
                                nonlocal threadMutex

                                try:
                                    robot = createRobot(nodeNr, port)

                                    t = 0.0
                                    doneRunning = False
                                    pwm = 0

                                    def sendCommandHandlerFunction(dt, robot):
                                        nonlocal nodeNr
                                        nonlocal t
                                        nonlocal maxOscillationFrq
                                        nonlocal maxPwmValue
                                        nonlocal testPwmValue
                                        nonlocal threadMutex
                                        nonlocal pwm

                                        servo = robot.dcServoArray[nodeNr - 1]

                                        frq = 0
                                        pwmAmp = 0
                                        with threadMutex:
                                            pwmAmp = testPwmValue
                                            frq = pwmAmp / maxPwmValue * maxOscillationFrq

                                        pwm = pwmAmp * math.sin(frq * 2 * pi * t)
                                        servo.setOpenLoopControlSignal(pwm, True)

                                    out = []

                                    lastPosition = None

                                    def readResultHandlerFunction(dt, robot):
                                        nonlocal nodeNr
                                        nonlocal t
                                        nonlocal runThread
                                        nonlocal doneRunning
                                        nonlocal pwm
                                        nonlocal lastPosition

                                        t += dt
                                        servo = robot.dcServoArray[nodeNr - 1]

                                        position = servo.getPosition(False)

                                        if lastPosition == None:
                                            lastPosition = position

                                        out.append([t, (position - lastPosition) / dt, pwm])

                                        lastPosition = position

                                        stop = False
                                        with threadMutex:
                                            if runThread == False:
                                                stop = True

                                        if stop:
                                            robot.removeHandlerFunctions()
                                            doneRunning = True

                                    robot.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction);

                                    while not doneRunning:
                                        time.sleep(0.1)

                                    robot.shutdown()

                                    data = np.array(out)

                                    def plotData(data):
                                        dialog = Gtk.MessageDialog(
                                                transient_for=self,
                                                flags=0,
                                                message_type=Gtk.MessageType.INFO,
                                                buttons=Gtk.ButtonsType.YES_NO,
                                                text='Pwm test done!',
                                        )
                                        dialog.format_secondary_text(
                                            "Do you want to plot the recorded data?"
                                        )
                                        response = dialog.run()
                                        dialog.destroy()

                                        if response == Gtk.ResponseType.YES:
                                            plt.figure(1)
                                            plt.plot(data[:, 0], data[:, 1], 'r')
                                            plt.figure(2)
                                            plt.plot(data[:, 0], data[:, 2], 'b')
                                            plt.show()

                                    GLib.idle_add(plotData, data)
                                except Exception as e:
                                    print(format(e))

                                GLib.idle_add(resetGuiAfterCalibration)

                            testThread = None

                            def onTestPwm(widget):
                                nonlocal calibrationBox
                                nonlocal startButton

                                nonlocal testThread
                                nonlocal runThread
                                nonlocal pwmValue
                                nonlocal threadMutex

                                if widget.get_label() == 'Test pwm value':
                                    startButton[1].set_sensitive(False)
                                    widget.set_label('Stop pwm test')
                                    with threadMutex:
                                        runThread = True
                                    nodeNr = getNodeNrFromCombo(activeNodeNrCombo[1])
                                    port = getComPortFromCombo(activeComPortCombo[1])
                                    testThread = threading.Thread(target=testPwmRun, args=(nodeNr,port,))
                                    testThread.start()
                                else:
                                    with threadMutex:
                                        runThread = False
                                    testThread.join()

                            def updateRecordingProgressBar(fraction):
                                nonlocal recordingProgressBar

                                recordingProgressBar[1].set_fraction(fraction)

                            def handleResults(data):
                                pwmNonlinearityIdentifier = PwmNonlinearityIdentifier(data)

                                configClassName = getConfigClassNameFromCombo(activeNodeNrCombo[1])

                                dialog = Gtk.MessageDialog(
                                        transient_for=self,
                                        flags=0,
                                        message_type=Gtk.MessageType.INFO,
                                        buttons=Gtk.ButtonsType.YES_NO,
                                        text='Pwm linearity calibration done!',
                                )
                                dialog.format_secondary_text(
                                    "Should the configuration be updated with the new data?"
                                )
                                pwmNonlinearityIdentifier.plotGeneratedVector(dialog.get_message_area())
                                response = dialog.run()
                                dialog.destroy()

                                if response == Gtk.ResponseType.YES:
                                    with open("config/" + configName, "r") as configFile:
                                        configFileAsString = configFile.read()

                                        classPattern =re.compile(r'(\s*class\s+' + configClassName + r'\s+(.*\n)*?(.*\n)*?\})') 
                                        temp = classPattern.search(configFileAsString)
                                        if temp != None:
                                            classString = temp.group(0)

                                            classString = pwmNonlinearityIdentifier.writeLinearizationFunctionToConfigClassString(classString)
                                            if classString != '':
                                                configFileAsString = re.sub(classPattern, classString, configFileAsString)
                                                with open("config/" + configName, "w") as configFile:
                                                    configFile.write(configFileAsString)
                                                    transferToTargetMessage()

                                                    return
                                        
                                    dialog = Gtk.MessageDialog(
                                            transient_for=self,
                                            flags=0,
                                            message_type=Gtk.MessageType.ERROR,
                                            buttons=Gtk.ButtonsType.OK,
                                            text='Configuration format error!',
                                    )
                                    dialog.format_secondary_text(
                                        "Please past in the linearization function manually"
                                    )
                                    box = dialog.get_message_area()
                                    funEntry = Gtk.Entry()
                                    funEntry.set_text(pwmNonlinearityIdentifier.getLinearizationFunction())
                                    box.add(funEntry)
                                    box.show_all()
                                    response = dialog.run()
                                    dialog.destroy()

                            def startCalibrationRun(nodeNr, port):
                                nonlocal runThread
                                nonlocal threadMutex
                                nonlocal maxOscillationFrq
                                nonlocal maxPwmValue

                                try:
                                    robot = createRobot(nodeNr, port)

                                    with threadMutex:
                                        highResStep = min(25, int(midPwmValue / 10.0))
                                        temp = [v for v in range(0, int(midPwmValue), highResStep)]
                                        print(temp)
                                        temp = np.array(temp)
                                        temp += highResStep
                                        temp = temp / temp[-1] * midPwmValue

                                        temp2 = [v for v in range(0, int(maxPwmValue - midPwmValue), 100)]
                                        temp2 = np.array(temp2)
                                        temp2 += 100
                                        temp2 = temp2 / temp2[-1] * (maxPwmValue - midPwmValue)
                                        temp2 += midPwmValue
                                        pwmSampleValues = np.append(temp, temp2)

                                        temp = []
                                        for v in pwmSampleValues:
                                            frq = v / maxPwmValue * maxOscillationFrq

                                            sampleTime = max(4.0, 2.0 + 10 / frq)

                                            i = 0
                                            while i < sampleTime:
                                                i += 1
                                                temp.append(v)

                                        pwmSampleValues = np.array(temp)
                                        print(pwmSampleValues)

                                    t = 0.0
                                    runTime = 6.0
                                    doneRunning = False

                                    def sendCommandHandlerFunction(dt, robot):
                                        nonlocal nodeNr
                                        nonlocal t
                                        nonlocal threadMutex
                                        nonlocal maxOscillationFrq
                                        nonlocal maxPwmValue

                                        t += dt

                                        servo = robot.dcServoArray[nodeNr - 1]

                                        pwmAmp = 0.0
                                        if int(t) < len(pwmSampleValues):
                                            pwmAmp = pwmSampleValues[int(t)]
                                        else:
                                            return

                                        frq = 0.0
                                        with threadMutex:
                                            frq = pwmAmp / maxPwmValue * maxOscillationFrq

                                        GLib.idle_add(updateRecordingProgressBar, t / len(pwmSampleValues))

                                        pwm = pwmAmp * math.sin(frq * 2 * pi * t)
                                        servo.setOpenLoopControlSignal(pwm, True)

                                    out = []

                                    lastPosition = None

                                    def readResultHandlerFunction(dt, robot):
                                        nonlocal nodeNr
                                        nonlocal t
                                        nonlocal runThread
                                        nonlocal doneRunning
                                        nonlocal pwmSampleValues
                                        nonlocal lastPosition

                                        stop = int(t) >= len(pwmSampleValues)
                                        with threadMutex:
                                            if runThread == False:
                                                stop = True

                                        if stop:
                                            robot.removeHandlerFunctions()
                                            doneRunning = True
                                            return

                                        servo = robot.dcServoArray[nodeNr - 1]

                                        position = servo.getPosition(False)

                                        if lastPosition == None:
                                            lastPosition = position

                                        out.append([t, (position - lastPosition) / dt, pwmSampleValues[int(t)]])

                                        lastPosition = position

                                    robot.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction);

                                    while not doneRunning:
                                        time.sleep(0.1)

                                    robot.shutdown()

                                    if runThread == True:
                                        data = np.array(out)
                                        GLib.idle_add(handleResults, data)

                                except Exception as e:
                                    print(format(e))

                                GLib.idle_add(resetGuiAfterCalibration)

                            def onStartCalibration(widget):
                                nonlocal testThread
                                nonlocal threadMutex
                                nonlocal runThread

                                nonlocal calibrationBox
                                nonlocal maxFrqScale
                                nonlocal midPwmScale
                                nonlocal maxPwmScale
                                nonlocal recordingProgressBar
                                nonlocal testButton

                                if widget.get_label() == 'Start calibration':
                                    configClassName = getConfigClassNameFromCombo(activeNodeNrCombo[1])

                                    with open("config/" + configName, "r") as configFile:
                                        configFileAsString = configFile.read()

                                        classPattern =re.compile(r'(\s*class\s+' + configClassName + r'\s+(.*\n)*?(.*\n)*?\})') 
                                        classString = classPattern.search(configFileAsString).group(0)

                                        linearizeVecPattern = re.compile(r'(\n([ \t]*).*createCurrentController\(\)\s*\{(.*\n)*?(\s*)auto\s+pwmHighFrqCompFun\s+=\s+\[\]\(uint16_t\s+in\)\4\{\n)([ \t]*)((.*\n)*?.*)(\4\};(.*\n)*?\2\})')

                                        if linearizeVecPattern.search(classString).group(6) != 'return in;':
                                            dialog = Gtk.MessageDialog(
                                                    transient_for=self,
                                                    flags=0,
                                                    message_type=Gtk.MessageType.ERROR,
                                                    buttons=Gtk.ButtonsType.YES_NO,
                                                    text='Pwm calibration already done for this configuration!',
                                            )
                                            dialog.format_secondary_text(
                                                "Pwm linarization only works on configurations without previous calibration.\n\nShould the calibration be reset?"
                                            )
                                            response = dialog.run()
                                            dialog.destroy()

                                            if response == Gtk.ResponseType.NO:
                                                return

                                            classString = re.sub(linearizeVecPattern, r'\1\5return in;\8', classString)
                                            configFileAsString = re.sub(classPattern, classString, configFileAsString)
                                            with open("config/" + configName, "w") as configFile:
                                                configFile.write(configFileAsString)
                                                transferToTargetMessage()

                                            return

                                    widget.set_label('Abort calibration')

                                    recordingProgressBar[1].set_fraction(0.0)
                                    testButton[1].set_sensitive(False)
                                    calibrationBox.pack_start(recordingProgressBar[0], False, False, 0)
                                    maxFrqScale[1].set_sensitive(False)
                                    midPwmScale[1].set_sensitive(False)
                                    maxPwmScale[1].set_sensitive(False)

                                    calibrationBox.show_all()

                                    with threadMutex:
                                        runThread = True
                                    nodeNr = getNodeNrFromCombo(activeNodeNrCombo[1])
                                    port = getComPortFromCombo(activeComPortCombo[1])
                                    testThread = threading.Thread(target=startCalibrationRun, args=(nodeNr, port,))
                                    testThread.start()                                    
                                else:
                                    with threadMutex:
                                        runThread = False
                                    testThread.join()

                            testButton[1].connect('clicked', onTestPwm)
                            startButton[1].connect('clicked', onStartCalibration)
                            calibrationBox.pack_start(testButton[0], False, False, 0)
                            calibrationBox.pack_start(startButton[0], False, False, 0)
                            calibrationBox.show_all()

                        elif calibrationType == 'System identification':
############################# System identification ################################################################
                            disconnectMotorFromGearboxMessage()

                            calibrationBox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
                            calibrationBox.set_margin_start(40)

                            motorSettleTime = 2
                            minPwmValue = 10
                            maxPwmValue = 100

                            motorSettleTimeScale = creatHScale(motorSettleTime, 1, 10, 1, getLowLev=True)
                            motorSettleTimeScale = addTopLabelTo('<b>Motor settle time</b>', motorSettleTimeScale[0]), motorSettleTimeScale[1]
                            calibrationBox.pack_start(motorSettleTimeScale[0], False, False, 0)

                            minPwmScale = creatHScale(minPwmValue, 0, 200, 1, getLowLev=True)
                            minPwmScale = addTopLabelTo('<b>Min motor pwm value</b>', minPwmScale[0]), minPwmScale[1]
                            calibrationBox.pack_start(minPwmScale[0], False, False, 0)

                            maxPwmScale = creatHScale(maxPwmValue, 0, 200, 1, getLowLev=True)
                            maxPwmScale = addTopLabelTo('<b>Max motor pwm value</b>', maxPwmScale[0]), maxPwmScale[1]
                            calibrationBox.pack_start(maxPwmScale[0], False, False, 0)

                            testButton = createButton('Test pwm value', getLowLev=True)
                            startButton = createButton('Start calibration', getLowLev=True)

                            recordingProgressBar = creatProgressBar(label='Recording', getLowLev=True)

                            testPwmValue = minPwmValue

                            threadMutex = threading.Lock()

                            def motorSettleTimeChanged(widget):
                                nonlocal motorSettleTime
                                nonlocal threadMutex

                                with threadMutex:
                                    motorSettleTime = widget.get_value()

                            def minPwmValueChanged(widget):
                                nonlocal maxPwmScale
                                nonlocal minPwmValue
                                nonlocal testPwmValue
                                nonlocal threadMutex

                                with threadMutex:
                                    minPwmValue = widget.get_value()
                                    testPwmValue = minPwmValue

                                if minPwmValue > maxPwmScale[1].get_value():
                                    maxPwmScale[1].set_value(minPwmValue)

                            def maxPwmValueChanged(widget):
                                nonlocal minPwmScale
                                nonlocal maxPwmValue
                                nonlocal testPwmValue
                                nonlocal threadMutex

                                with threadMutex:
                                    maxPwmValue = widget.get_value()
                                    testPwmValue = maxPwmValue

                                if maxPwmValue < minPwmScale[1].get_value():
                                    minPwmScale[1].set_value(maxPwmValue)

                            motorSettleTimeScale[1].connect('value-changed', motorSettleTimeChanged)
                            minPwmScale[1].connect('value-changed', minPwmValueChanged)
                            maxPwmScale[1].connect('value-changed', maxPwmValueChanged)

                            def resetGuiAfterCalibration():
                                nonlocal startButton
                                nonlocal testButton
                                nonlocal calibrationBox
                                nonlocal recordingProgressBar
                                nonlocal minPwmScale
                                nonlocal maxPwmScale

                                testButton[1].set_label('Test pwm value')
                                testButton[1].set_sensitive(True)
                                startButton[1].set_label('Start calibration')
                                startButton[1].set_sensitive(True)
                                calibrationBox.remove(recordingProgressBar[0])
                                motorSettleTimeScale[1].set_sensitive(True)
                                minPwmScale[1].set_sensitive(True)
                                maxPwmScale[1].set_sensitive(True)

                            runThread = False
                            def testPwmRun(nodeNr, port):
                                nonlocal runThread
                                nonlocal threadMutex

                                try:
                                    robot = createRobot(nodeNr, port)

                                    t = 0.0
                                    doneRunning = False

                                    def sendCommandHandlerFunction(dt, robot):
                                        nonlocal nodeNr
                                        nonlocal t
                                        nonlocal testPwmValue
                                        nonlocal threadMutex

                                        servo = robot.dcServoArray[nodeNr - 1]

                                        servo.setOpenLoopControlSignal(testPwmValue, True)

                                    out = []

                                    def readResultHandlerFunction(dt, robot):
                                        nonlocal nodeNr
                                        nonlocal t
                                        nonlocal runThread
                                        nonlocal doneRunning

                                        t += dt
                                        servo = robot.dcServoArray[nodeNr - 1]
                                        out.append([t, servo.getVelocity()])

                                        stop = False
                                        with threadMutex:
                                            if runThread == False:
                                                stop = True

                                        if stop:
                                            robot.removeHandlerFunctions()
                                            doneRunning = True

                                    robot.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction);

                                    while not doneRunning:
                                        time.sleep(0.1)

                                    robot.shutdown()

                                    data = np.array(out)

                                    def plotData(data):
                                        dialog = Gtk.MessageDialog(
                                                transient_for=self,
                                                flags=0,
                                                message_type=Gtk.MessageType.INFO,
                                                buttons=Gtk.ButtonsType.YES_NO,
                                                text='Pwm test done!',
                                        )
                                        dialog.format_secondary_text(
                                            "Do you want to plot the recorded data?"
                                        )
                                        response = dialog.run()
                                        dialog.destroy()

                                        if response == Gtk.ResponseType.YES:
                                            plt.figure(1)
                                            plt.plot(data[:, 0], data[:, 1], 'r')
                                            plt.show()

                                    GLib.idle_add(plotData, data)
                                except Exception as e:
                                    print(format(e))

                                GLib.idle_add(resetGuiAfterCalibration)

                            testThread = None

                            def onTestPwm(widget):
                                nonlocal calibrationBox
                                nonlocal startButton

                                nonlocal testThread
                                nonlocal runThread
                                nonlocal pwmValue
                                nonlocal threadMutex

                                if widget.get_label() == 'Test pwm value':
                                    startButton[1].set_sensitive(False)
                                    widget.set_label('Stop pwm test')
                                    with threadMutex:
                                        runThread = True
                                    nodeNr = getNodeNrFromCombo(activeNodeNrCombo[1])
                                    port = getComPortFromCombo(activeComPortCombo[1])
                                    testThread = threading.Thread(target=testPwmRun, args=(nodeNr, port,))
                                    testThread.start()
                                else:
                                    with threadMutex:
                                        runThread = False
                                    testThread.join()

                            def updateRecordingProgressBar(fraction):
                                nonlocal recordingProgressBar

                                recordingProgressBar[1].set_fraction(fraction)

                            def handleResults(data):
                                configClassName = getConfigClassNameFromCombo(activeNodeNrCombo[1])

                                systemIdentifier = SystemIdentificationObject(data)

                                dialog = Gtk.MessageDialog(
                                        transient_for=self,
                                        flags=0,
                                        message_type=Gtk.MessageType.INFO,
                                        buttons=Gtk.ButtonsType.YES_NO,
                                        text='System identification done!',
                                )
                                dialog.format_secondary_text(
                                    "Should the configuration be updated with the new model?"
                                )
                                systemIdentifier.plotServoSystemModel(dialog.get_message_area())
                                response = dialog.run()
                                dialog.destroy()

                                if response == Gtk.ResponseType.YES:
                                    dt = 0.0012
                                    with open("config/" + configName, "r") as configFile:
                                        configFileAsString = configFile.read()

                                        classPattern =re.compile(r'(\s*class\s+' + configClassName + r'\s+(.*\n)*?(.*\n)*?\})') 
                                        temp = classPattern.search(configFileAsString)
                                        if temp != None:
                                            classString = temp.group(0)

                                            dtPattern = re.compile(r'Eigen::Matrix3f\s+A;\s+A\s*<<\s*[^,]*,\s*([\d.]*)')
                                            temp = dtPattern.search(classString)
                                            if temp != None:
                                                dt = float(temp.group(1))

                                                servoModel = ServoModel(dt, systemIdentifier)
                                                
                                                classString = servoModel.writeModelToConfigClassString(classString)

                                                if classString != '':
                                                    configFileAsString = re.sub(classPattern, classString, configFileAsString)
                                                    with open("config/" + configName, "w") as configFile:
                                                        configFile.write(configFileAsString)
                                                        transferToTargetMessage()

                                                        return

                                    dialog = Gtk.MessageDialog(
                                            transient_for=self,
                                            flags=0,
                                            message_type=Gtk.MessageType.ERROR,
                                            buttons=Gtk.ButtonsType.OK,
                                            text='Configuration format error!',
                                    )
                                    dialog.format_secondary_text(
                                        "Please past in the new model manually"
                                    )
                                    box = dialog.get_message_area()
                                    funEntry = Gtk.Entry()
                                    funEntry.set_text(ServoModel(dt, systemIdentifier).getGeneratedModel())
                                    box.add(funEntry)
                                    box.show_all()
                                    response = dialog.run()
                                    dialog.destroy()

                            def startCalibrationRun(nodeNr, port):
                                nonlocal runThread
                                nonlocal threadMutex
                                nonlocal minPwmValue
                                nonlocal maxPwmValue
                                nonlocal motorSettleTime

                                try:
                                    robot = createRobot(nodeNr, port, 0.018)

                                    pwmSampleValues = []
                                    nr = 10
                                    for i in range(1, nr + 1):
                                        pwmSampleValues.append(i * (maxPwmValue - minPwmValue) / nr + minPwmValue)
                                        pwmSampleValues.append(minPwmValue)
                                        pwmSampleValues.append(-(i * (maxPwmValue - minPwmValue) / nr + minPwmValue))
                                        pwmSampleValues.append(-minPwmValue)

                                    t = 0.0
                                    doneRunning = False

                                    def sendCommandHandlerFunction(dt, robot):
                                        nonlocal nodeNr
                                        nonlocal t
                                        nonlocal threadMutex
                                        nonlocal maxOscillationFrq
                                        nonlocal maxPwmValue

                                        t += dt

                                        servo = robot.dcServoArray[nodeNr - 1]

                                        pwm = 0.0
                                        if int(t / motorSettleTime) < len(pwmSampleValues):
                                            pwm = pwmSampleValues[int(t / motorSettleTime)]
                                        else:
                                            return

                                        GLib.idle_add(updateRecordingProgressBar, (t / motorSettleTime) / len(pwmSampleValues))

                                        servo.setOpenLoopControlSignal(pwm, True)

                                    out = []

                                    def readResultHandlerFunction(dt, robot):
                                        nonlocal nodeNr
                                        nonlocal t
                                        nonlocal runThread
                                        nonlocal doneRunning
                                        nonlocal pwmSampleValues

                                        stop = int(t / motorSettleTime) >= len(pwmSampleValues)
                                        with threadMutex:
                                            if runThread == False:
                                                stop = True

                                        if stop:
                                            robot.removeHandlerFunctions()
                                            doneRunning = True
                                            return

                                        servo = robot.dcServoArray[nodeNr - 1]
                                        out.append([t,
                                                servo.getPosition(False) / servo.getScaling(),
                                                pwmSampleValues[int(t / motorSettleTime)]])

                                    robot.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction);

                                    while not doneRunning:
                                        time.sleep(0.1)

                                    robot.shutdown()

                                    if runThread == True:
                                        data = np.array(out)
                                        GLib.idle_add(handleResults, data)

                                except Exception as e:
                                    print(format(e))

                                GLib.idle_add(resetGuiAfterCalibration)

                            def onStartCalibration(widget):
                                nonlocal testThread
                                nonlocal threadMutex
                                nonlocal runThread

                                nonlocal calibrationBox
                                nonlocal motorSettleTimeScale
                                nonlocal minPwmScale
                                nonlocal maxPwmScale
                                nonlocal recordingProgressBar
                                nonlocal testButton

                                if widget.get_label() == 'Start calibration':
                                    widget.set_label('Abort calibration')

                                    recordingProgressBar[1].set_fraction(0.0)
                                    testButton[1].set_sensitive(False)
                                    calibrationBox.pack_start(recordingProgressBar[0], False, False, 0)
                                    motorSettleTimeScale[1].set_sensitive(False)
                                    minPwmScale[1].set_sensitive(False)
                                    maxPwmScale[1].set_sensitive(False)

                                    calibrationBox.show_all()

                                    with threadMutex:
                                        runThread = True
                                    nodeNr = getNodeNrFromCombo(activeNodeNrCombo[1])
                                    port = getComPortFromCombo(activeComPortCombo[1])
                                    testThread = threading.Thread(target=startCalibrationRun, args=(nodeNr, port,))
                                    testThread.start()                                    
                                else:
                                    with threadMutex:
                                        runThread = False
                                    testThread.join()

                            testButton[1].connect('clicked', onTestPwm)
                            startButton[1].connect('clicked', onStartCalibration)
                            calibrationBox.pack_start(testButton[0], False, False, 0)
                            calibrationBox.pack_start(startButton[0], False, False, 0)
                            calibrationBox.show_all()

                        elif calibrationType == 'Output encoder calibration':
############################# Output encoder calibration ################################################################
                            calibrationBox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
                            calibrationBox.set_margin_start(40)

                            startButton = createButton('Start calibration', getLowLev=True)

                            recordingProgressBar = creatProgressBar(label='Recording', getLowLev=True)

                            threadMutex = threading.Lock()

                            def resetGuiAfterCalibration():
                                nonlocal startButton
                                nonlocal testButton
                                nonlocal calibrationBox
                                nonlocal recordingProgressBar
                                nonlocal minPwmScale
                                nonlocal maxPwmScale

                                startButton[1].set_label('Start calibration')
                                startButton[1].set_sensitive(True)
                                calibrationBox.remove(recordingProgressBar[0])

                            runThread = False

                            def updateRecordingProgressBar(fraction):
                                nonlocal recordingProgressBar

                                recordingProgressBar[1].set_fraction(fraction)

                            def handleResults(data):
                                configClassName = getConfigClassNameFromCombo(activeNodeNrCombo[1])

                                with open("config/" + configName, "r") as configFile:
                                    configFileAsString = configFile.read()

                                    classPattern =re.compile(r'(\s*class\s+' + configClassName + r'\s+(.*\n)*?(.*\n)*?\})') 
                                    classString = classPattern.search(configFileAsString).group(0)

                                    wrapAroundAndUnitPerRevPattern = re.compile(r'return\s+std::make_unique\s*<\s*(\w*)\s*>\s*\(([^\)]*)\s*compVec\s*\)\s*;')

                                    temp = wrapAroundAndUnitPerRevPattern.search(classString)
                                    
                                    magneticEncoder = temp.group(1) == 'EncoderHandler'
                                    unitsPerRev = 4096
                                    if not magneticEncoder:
                                        paramStr = wrapAroundAndUnitPerRevPattern.search(classString).group(2)
                                        i = paramStr.find(',')
                                        paramStr = paramStr[i + 1:]
                                        i = paramStr.find(',')
                                        paramStr = paramStr[0:i]
                                        unitsPerRev = eval(paramStr)

                                    outputEncoderCalibrationGenerator = OutputEncoderCalibrationGenerator(data, magneticEncoder, unitsPerRev)

                                    dialog = Gtk.MessageDialog(
                                            transient_for=self,
                                            flags=0,
                                            message_type=Gtk.MessageType.INFO,
                                            buttons=Gtk.ButtonsType.YES_NO,
                                            text='Output encoder calibration done!',
                                    )
                                    dialog.format_secondary_text(
                                        "Should the configuration be updated with the green compensation vector?"
                                    )
                                    outputEncoderCalibrationGenerator.plotGeneratedVector(dialog.get_message_area())
                                    response = dialog.run()
                                    dialog.destroy()

                                    if response == Gtk.ResponseType.YES:
                                        classString = outputEncoderCalibrationGenerator.writeVectorToConfigClassString(classString)

                                        if classString != '':
                                            configFileAsString = re.sub(classPattern, classString, configFileAsString)
                                            with open("config/" + configName, "w") as configFile:
                                                configFile.write(configFileAsString)
                                                transferToTargetMessage()

                                                return

                                        dialog = Gtk.MessageDialog(
                                                transient_for=self,
                                                flags=0,
                                                message_type=Gtk.MessageType.ERROR,
                                                buttons=Gtk.ButtonsType.OK,
                                                text='Configuration format error!',
                                        )
                                        dialog.format_secondary_text(
                                            "Please past in the new compensation vector manually"
                                        )
                                        box = dialog.get_message_area()
                                        vecEntry = Gtk.Entry()
                                        vecEntry.set_text(outputEncoderCalibrationGenerator.getGeneratedModel())
                                        box.add(vecEntry)
                                        box.show_all()
                                        response = dialog.run()
                                        dialog.destroy()

                            def startCalibrationRun(nodeNr, port):
                                nonlocal runThread
                                nonlocal threadMutex

                                try:
                                    robot = createRobot(nodeNr, port)

                                    t = 0.0
                                    doneRunning = False

                                    def sendCommandHandlerFunction(dt, robot):
                                        nonlocal nodeNr
                                        nonlocal t
                                        nonlocal threadMutex

                                        servo = robot.dcServoArray[nodeNr - 1]

                                        servo.setOpenLoopControlSignal(0, True)

                                    out = []

                                    def readResultHandlerFunction(dt, robot):
                                        nonlocal nodeNr
                                        nonlocal t
                                        nonlocal runThread
                                        nonlocal doneRunning

                                        runTime = 120.0

                                        t += dt

                                        stop = t > runTime
                                        with threadMutex:
                                            if runThread == False:
                                                stop = True

                                        if stop:
                                            robot.removeHandlerFunctions()
                                            doneRunning = True
                                            return

                                        servo = robot.dcServoArray[nodeNr - 1]
                                        out.append([t,
                                                (servo.getPosition(True) - servo.getOffset()) / servo.getScaling(),
                                                (servo.getPosition(True) - servo.getPosition(False)) / servo.getScaling()])

                                        GLib.idle_add(updateRecordingProgressBar, t / runTime)

                                    robot.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction);

                                    while not doneRunning:
                                        time.sleep(0.1)

                                    robot.shutdown()

                                    if runThread == True:
                                        data = np.array(out)
                                        GLib.idle_add(handleResults, data)

                                except Exception as e:
                                    print(format(e))

                                GLib.idle_add(resetGuiAfterCalibration)

                            def onStartCalibration(widget):
                                nonlocal testThread
                                nonlocal threadMutex
                                nonlocal runThread

                                nonlocal calibrationBox
                                nonlocal minPwmScale
                                nonlocal maxPwmScale
                                nonlocal recordingProgressBar
                                nonlocal testButton

                                if widget.get_label() == 'Start calibration':
                                    configClassName = getConfigClassNameFromCombo(activeNodeNrCombo[1])

                                    with open("config/" + configName, "r") as configFile:
                                        configFileAsString = configFile.read()

                                        classPattern =re.compile(r'(\s*class\s+' + configClassName + r'\s+(.*\n)*?(.*\n)*?\})')
                                        temp = classPattern.search(configFileAsString)
                                        if temp != None:
                                            classString = temp.group(0)

                                            compVecPattern = re.compile(r'(.*createOutputEncoderHandler\(\)\s*\{(.*\n)*?\s*std\s*::\s*array\s*<\s*int16_t\s*,\s*513\s*>\s*compVec\s*=\s*)\{\s*([^\}]*)\s*\};')
            
                                            temp = compVecPattern.search(configClassString)
                                            if temp != None:
                                                if temp.group(3) != '0':
                                                    dialog = Gtk.MessageDialog(
                                                            transient_for=self,
                                                            flags=0,
                                                            message_type=Gtk.MessageType.ERROR,
                                                            buttons=Gtk.ButtonsType.YES_NO,
                                                            text='Output encoder calibration already done for this configuration!',
                                                    )
                                                    dialog.format_secondary_text(
                                                        "Output encoder calibration only works on configurations without previous calibration.\n\nShould the calibration be reset?"
                                                    )
                                                    response = dialog.run()
                                                    dialog.destroy()

                                                    if response == Gtk.ResponseType.NO:
                                                        return

                                                    classString = re.sub(compVecPattern, r'\1{0};', classString)
                                                    configFileAsString = re.sub(classPattern, classString, configFileAsString)
                                                    with open("config/" + configName, "w") as configFile:
                                                        configFile.write(configFileAsString)
                                                        transferToTargetMessage()

                                                    return

                                    widget.set_label('Abort calibration')

                                    startManuallyCalibrationMessage('60 seconds')

                                    recordingProgressBar[1].set_fraction(0.0)
                                    calibrationBox.pack_start(recordingProgressBar[0], False, False, 0)

                                    calibrationBox.show_all()

                                    with threadMutex:
                                        runThread = True
                                    nodeNr = getNodeNrFromCombo(activeNodeNrCombo[1])
                                    port = getComPortFromCombo(activeComPortCombo[1])
                                    testThread = threading.Thread(target=startCalibrationRun, args=(nodeNr, port,))
                                    testThread.start()                                    
                                else:
                                    with threadMutex:
                                        runThread = False
                                    testThread.join()

                            startButton[1].connect('clicked', onStartCalibration)
                            calibrationBox.pack_start(startButton[0], False, False, 0)
                            calibrationBox.show_all()

                        elif calibrationType == 'Test control loop':
############################# Test control loop ################################################################
                            calibrationBox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
                            calibrationBox.set_margin_start(40)


                            controlSpeedScale = creatHScale(14, 0, 100, 1, getLowLev=True)
                            controlSpeedScale = addTopLabelTo('<b>Control speed</b>', controlSpeedScale[0]), controlSpeedScale[1]
                            calibrationBox.pack_start(controlSpeedScale[0], False, False, 0)

                            backlashControlSpeedScale = creatHScale(0, 0, 50, 1, getLowLev=True)
                            backlashControlSpeedScale = addTopLabelTo('<b>Backlash control speed</b>', backlashControlSpeedScale[0]), backlashControlSpeedScale[1]
                            calibrationBox.pack_start(backlashControlSpeedScale[0], False, False, 0)

                            testVel = 0

                            testVelScale = creatHScale(0.0, -1.000, 1.000, 0.010, getLowLev=True)
                            testVelScale = addTopLabelTo('<b>Velocity</b>\n in radians per second', testVelScale[0]), testVelScale[1]
                            startButton = createButton('Start test', getLowLev=True)

                            positionOffsetScale = creatHScale(0.0, -1.0, 1.0, 0.010, getLowLev=True)
                            positionOffsetScale = addTopLabelTo('<b>Position offset</b>\n in radians', positionOffsetScale[0]), positionOffsetScale[1]
                            calibrationBox.pack_start(positionOffsetScale[0], False, False, 0)

                            def onTestVelScaleChange(widget):
                                nonlocal threadMutex
                                nonlocal testVel

                                with threadMutex:
                                    testVel = widget.get_value()

                            testVelScale[1].connect('value-changed', onTestVelScaleChange)

                            threadMutex = threading.Lock()

                            def resetGuiAfterCalibration():
                                nonlocal startButton
                                nonlocal testButton
                                nonlocal calibrationBox
                                nonlocal recordingProgressBar
                                nonlocal minPwmScale
                                nonlocal maxPwmScale

                                startButton[1].set_label('Start test')
                                startButton[1].set_sensitive(True)

                            runThread = False

                            def handleResults(data):
                                plt.figure(1)
                                plt.plot(data[:, 0], data[:, 1])

                                plt.figure(2)
                                plt.plot(data[:, 0], data[:, 2])

                                plt.figure(3)
                                plt.plot(data[:, 0], data[:, 3])
                                plt.show()

                            def startTestRun(nodeNr, port):
                                nonlocal runThread
                                nonlocal threadMutex
                                nonlocal testVel

                                controlSpeed = int(controlSpeedScale[1].get_value())
                                backlashControlSpeed = int(backlashControlSpeedScale[1].get_value())

                                positionOffset = positionOffsetScale[1].get_value()

                                try:
                                    def initFun(robot):
                                        robot.dcServoArray[nodeNr - 1].setControlSpeed(controlSpeed)
                                        robot.dcServoArray[nodeNr - 1].setBacklashControlSpeed(backlashControlSpeed, 3.0, 0.0)

                                    robot = createRobot(nodeNr, port, dt=0.018, initFunction=initFun)

                                    t = 0.0
                                    doneRunning = False

                                    pos = robot.dcServoArray[nodeNr - 1].getPosition() + positionOffset

                                    def sendCommandHandlerFunction(dt, robot):
                                        nonlocal nodeNr
                                        nonlocal t
                                        nonlocal threadMutex
                                        nonlocal pos
                                        nonlocal testVel

                                        servo = robot.dcServoArray[nodeNr - 1]

                                        vel = 0
                                        with threadMutex:
                                            vel = testVel

                                        servo.setReference(pos, vel, 0)

                                        pos += dt * vel

                                    out = []

                                    def readResultHandlerFunction(dt, robot):
                                        nonlocal nodeNr
                                        nonlocal t
                                        nonlocal runThread
                                        nonlocal doneRunning

                                        t += dt

                                        stop = False
                                        with threadMutex:
                                            if runThread == False:
                                                stop = True

                                        if stop:
                                            robot.removeHandlerFunctions()
                                            doneRunning = True
                                            return

                                        servo = robot.dcServoArray[nodeNr - 1]
                                        out.append([t,
                                                servo.getControlError(True),
                                                servo.getVelocity(),
                                                servo.getControlError(False)])

                                    robot.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction);

                                    while not doneRunning:
                                        time.sleep(0.1)

                                    robot.shutdown()

                                    data = np.array(out)
                                    GLib.idle_add(handleResults, data)

                                except Exception as e:
                                    print(format(e))

                                GLib.idle_add(resetGuiAfterCalibration)

                            testThread = None

                            def onStartCalibration(widget):
                                nonlocal testThread
                                nonlocal threadMutex
                                nonlocal runThread

                                nonlocal calibrationBox
                                nonlocal minPwmScale
                                nonlocal maxPwmScale
                                nonlocal recordingProgressBar
                                nonlocal testButton

                                if widget.get_label() == 'Start test':
                                    widget.set_label('Stop test')

                                    calibrationBox.show_all()

                                    with threadMutex:
                                        runThread = True
                                    nodeNr = getNodeNrFromCombo(activeNodeNrCombo[1])
                                    port = getComPortFromCombo(activeComPortCombo[1])
                                    testThread = threading.Thread(target=startTestRun, args=(nodeNr, port,))
                                    testThread.start()                                    
                                else:
                                    with threadMutex:
                                        runThread = False
                                    testThread.join()

                            calibrationBox.pack_start(testVelScale[0], False, False, 0)
                            startButton[1].connect('clicked', onStartCalibration)
                            calibrationBox.pack_start(startButton[0], False, False, 0)
                            calibrationBox.show_all()

                        if calibrationBox != None:
                            box1.pack_start(calibrationBox, False, False, 0)

                    calibrationCombo = creatComboBox('', supportedCalibrationOptions, onCaribrationTypeChange, getLowLev=True)
                    calibrationCombo = addTopLabelTo('<b>Supported calibrations</b>', calibrationCombo[0]), calibrationCombo[1]
                    box1.pack_start(calibrationCombo[0], False, False, 0)
                    box1.show_all()


        activeConfigCombo[1].connect('changed', onActiveConfigChange)

    def getConfigurations(self):
        configs = []
        path = 'config'
        for filename in os.listdir(path):
            ipath = path + '/' + filename
            if not os.path.isdir(ipath) and not filename == 'config.h' and filename[-2:] == '.h':
                configs.append(filename)

        configs.sort()
        return configs

    def getSelectedConfigurationName(self):
        if os.path.exists('config/config.h'):
            configFile = open('config/config.h', 'r')
            configName = configFile.read()
            configFile.close()
            startStr = '#include \''
            if configName.find(startStr) == 0:
                endStr = '\"'
                configName = configName[len(startStr):]
                endI = configName.find(endStr)
                if not endI == -1:
                    return configName[0:endI]
        return ''


if __name__ == "__main__":
    window = GuiWindow()
    window.connect("destroy", Gtk.main_quit)
    window.show_all()

    #work around for mathplotlib
    #calling Gtk.main_quit when all shown
    #figures are closed
    while not window.isClosed:
        Gtk.main()
