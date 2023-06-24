'''
Module for with common functions for CalibrationAnalyzers
'''

# pylint: disable=unused-import
import threading
import time
import math
import random
import re
import numpy as np
import scipy.signal
import matplotlib.pyplot as plt
from matplotlib.backends.backend_gtk3agg import (
    FigureCanvasGTK3Agg as FigureCanvas)
from matplotlib.figure import Figure
import ServoProjectModules.Communication as ServoComModule
from ServoProjectModules.Communication import pi
import ServoProjectModules.GuiHelper as GuiFunctions
from ServoProjectModules.GuiHelper import GLib, Gtk

def createServoManager(nodeNr, port, dt=0.004, initFunction=None):
    if port != '':
        com = ServoComModule.SerialCommunication(port)
    else:
        com = ServoComModule.SimulateCommunication(cycleTime=dt)

    def createServoFunction():
        nonlocal nodeNr
        nonlocal com
        servo = ServoComModule.DCServoCommunicator(nodeNr, com)

        servo.setOffsetAndScaling(2 * pi / 4096.0, 0.0, 0)

        servo.setControlSpeed(20)
        servo.setBacklashControlSpeed(0, 3.0, 0.00)
        servo.setFrictionCompensation(0)

        servoArray = [servo]

        if initFunction:
            initFunction(servoArray)

        return servoArray

    servoManager = ServoComModule.ServoManager(cycleTime=dt, initFunction=createServoFunction)

    return servoManager

def median(vec):
    if len(vec) == 0:
        return math.nan
    sortedVec = sorted(list(vec))
    i = len(vec)//2-1
    if len(vec) % 2 == 1:
        return sortedVec[i]
    return sum(sortedVec[i:i+2])/2

def meanIgnoringNan(x):
    skipNan = [v for v in x if not math.isnan(v)]
    return np.mean(skipNan)

def calcSpikeResistantAvarage(vec, excluded=0.33):
    if len(vec) == 0:
        return math.nan
    vec = sorted(vec)
    excludeIndex = int(len(vec) * excluded)
    if excludeIndex > 0:
        vec = vec[excludeIndex - 1:-excludeIndex]
    return sum(vec) / len(vec)

def slidingAvarageFiltering(y, filterN):
    y = list(y)

    if filterN <= 1:
        return y

    filterN = max(1, filterN // 2)
    return [meanIgnoringNan(
            y[min(i-filterN, 0):0]
            + y[max(i-filterN, 0):min(i+filterN+1, len(y))]
            + y[0:max(0, i+filterN+1-len(y))]) for i in range(0, len(y))]


def fftFilter(tt, yy, freqCut, minFreqCut=0, upSampleTt=None):
    mean = meanIgnoringNan(yy)
    yy = [mean if math.isnan(v) else v for v in yy]
    tt = np.array(tt)
    yy = np.array(yy)

    l = len(tt)
    ff = np.fft.fftfreq(l, (tt[1]-tt[0]))   # assume uniform spacing
    fyy = np.fft.fft(yy)

    for i, f in enumerate(ff):
        if abs(f) > freqCut:
            fyy[i] = 0.0
        elif abs(f) < minFreqCut:
            fyy[i] = 0.0

    if upSampleTt is None:
        return np.real(np.fft.ifft(fyy))

    def symbolicRepOfTtToYy(tt):
        #x[n] = 1/N * sum{k=0 to N-1} X[k] * exp(j * 2*pi * k * n / N)

        yy = tt * 0
        for f, fy in zip(ff, fyy):
            yy = yy + 1 / l * fy * np.exp(2j * math.pi * f * tt)

        return yy

    return np.real(symbolicRepOfTtToYy(upSampleTt))

def shrinkArray(a, size, useMedian = False):
    if len(a) <= size:
        return a

    newA = []

    indexScale = len(a) / size

    index = 0

    while index < len(a):
        nextIndex = index + indexScale
        if nextIndex > len(a):
            nextIndex = len(a)

        subA = a[int(index): int(nextIndex)]
        if useMedian:
            newA.append(median(subA))
        else:
            newA.append(sum(subA) / len(subA))
        index = nextIndex

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
            string += str(j) + "f"
    string += ";\n"
    return string

def sign(v):
    if v >= 0:
        return 1.0
    return -1.0

def getConfigClassString(configFileAsString, configClassName):
    classPattern = re.compile(r'(\s*class\s+' + configClassName + r'\s+(.*\n)*?(.*\n)*?\}(;\n+SimulationHandler\s+'
            + configClassName + r'::simHandler)?)')
    temp = classPattern.search(configFileAsString)
    if not temp:
        return ''

    return temp.group(0)

def setConfigClassString(configFileAsString, configClassName, classString):
    classPattern = re.compile(r'(\s*class\s+' + configClassName + r'\s+(.*\n)*?(.*\n)*?\})')
    temp = classPattern.search(configFileAsString)
    if not temp:
        return ''

    configFileAsString = re.sub(classPattern, classString, configFileAsString)

    return configFileAsString

def newConfigFileAsString(configClassString, nodeNr, configClassName):
    # pylint: disable=line-too-long
    out = ''
    out += '#ifndef CONFIG_HOLDER_H\n'
    out += '#define CONFIG_HOLDER_H\n'
    out += '\n'
    out += '#include "../defaultConfigHolder.h"'
    out += ''
    out += configClassString + ';\n'
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
    out += f'                std::make_unique<DCServoCommunicationHandler>({int(nodeNr)}, createDCServo<{configClassName}>()));\n'
    out += '\n'
    out += '        return com;\n'
    out += '    }\n'
    out += '};\n'
    out += '\n'
    out += '#endif\n'
    return out

wrapAroundAndUnitPerRevPattern = re.compile(
        r'(?P<beg>return\s+std::make_unique\s*<\s*)(?P<encoderType>\w*)'
        r'(?P<mid>\s*>\s*\((\w+\s*,\s*))(?P<units>[^;]*)(?P<end>,\s*compVec\s*\)\s*;)')

def getConfiguredOutputEncoderData(configClassString):
    temp = wrapAroundAndUnitPerRevPattern.search(configClassString)

    magneticEncoder = temp.group('encoderType') == 'EncoderHandler'
    unitsPerRev = 4096
    if not magneticEncoder:
        unitsStr = wrapAroundAndUnitPerRevPattern.search(configClassString).group('units')

        unitsStr = re.sub(r'f', '', unitsStr)
        unitsPerRev = eval(unitsStr)  # pylint: disable=eval-used

    return magneticEncoder, unitsPerRev

def setConfiguredOutputEncoderData(configClassString, magneticEncoder, unitsPerRev):
    encoderTypeStr = ('EncoderHandler' if magneticEncoder else 'ResistiveEncoderHandler')

    unitsStr = '4096.0f'
    if not magneticEncoder:
        unitsStr = f'4096.0f * {unitsPerRev / 4096 * 360 :0.1f}f / 360'

    configClassString = re.sub(wrapAroundAndUnitPerRevPattern,
            r'\g<beg>' + encoderTypeStr +
            r'\g<mid>' + unitsStr + r'\g<end>', configClassString)

    return configClassString

parmeterPattern = re.compile(
        r'(?P<beg>.*createMainEncoderHandler\(\)\s*\{(?:.*\n)*?\s*return\s+std::make_unique\s*<\s*(\w*)\s*>\s*\()'
        r'(?P<params>[^;]*)(?P<end>\s*\)\s*;)')
unitsPattern = re.compile(r'(?P<beg>(\w+\s*,\s*){4}\s*)(?P<units>.*)')
gearRatioPattern = re.compile(r'(4096\.0f?\s*\*\s*)(?P<gearRatio>.*)')

def getConfiguredGearRatio(configClassString):
    temp = parmeterPattern.search(configClassString)

    paramStr = temp.group('params')
    temp = unitsPattern.search(paramStr)

    unitsStr = temp.group('units')
    unitsStr = re.sub(r'f', '', unitsStr)

    gearRatioStr = ''
    temp = gearRatioPattern.search(unitsStr)
    if temp:
        gearRatioStr = temp.group('gearRatio')
        gearRatioStr = re.sub(r'f', '', gearRatioStr)

    if gearRatioStr == '':
        return str(eval(unitsStr) / 4096)  # pylint: disable=eval-used

    return gearRatioStr

def setConfiguredGearRatio(configClassString, gearRatioStr):
    temp = parmeterPattern.search(configClassString)
    if not temp:
        return ''

    gearRatioStr = re.sub(r'(?P<digit>\d\.\d+)(?P<end>\s*/)', r'\g<digit>f\g<end>', gearRatioStr)
    gearRatioStr = re.sub(r'(?P<digit>\d)(?P<end>\s*/)', r'\g<digit>.0f\g<end>', gearRatioStr)
    paramStr = temp.group('params')
    paramStr = re.sub(unitsPattern, r'\g<beg>4096.0f * ' + gearRatioStr, paramStr)
    configClassString = re.sub(parmeterPattern, r'\g<beg>' + paramStr + r'\g<end>', configClassString)

    return configClassString

class PwmNonlinearityConfigHandler:
    def __init__(self, pwmCompLookUp=None, pwmOffset=None):
        self.pwmCompLookUp = pwmCompLookUp
        self.pwmOffset = pwmOffset

    _linearizeFuncReturnPattern = re.compile(
            r'(\n([ \t]*).*createCurrentController\(\)\s*\{(.*\n)*?(\s*)auto\s+pwmHighFrqCompFun\s+=\s+'
            r'\[\]\(uint16_t\s+in\)\4\{\n)([ \t]*)(?P<function>(.*\n)*?.*)(\4\};(.*\n)*?\2\})')
    _linearizeVecPattern = re.compile(
            r'((\s*).*createCurrentController\(\)\s*\{(.*\n)*?)(\s*)auto\s+pwmHighFrqCompFun\s+=\s+'
            r'\[\]\(uint16_t\s+in\)(.*\n)*?\4\};((.*\n)*?\2\})')

    @staticmethod
    def checkForPreviousCalibration(configFileAsString, configClassName):
        configClassString = getConfigClassString(configFileAsString, configClassName)

        temp = PwmNonlinearityConfigHandler._linearizeFuncReturnPattern.search(configClassString)
        if not temp:
            raise Exception('Configuration not compatible')

        if temp.group('function') != 'return in;':
            return True

        return False

    @staticmethod
    def resetPreviousCalibration(configFileAsString, configClassName):
        configClassString = getConfigClassString(configFileAsString, configClassName)

        configClassString = re.sub(PwmNonlinearityConfigHandler._linearizeFuncReturnPattern, r'\1\5return in;\8',
                                    configClassString)
        configFileAsString = setConfigClassString(configFileAsString, configClassName, configClassString)

        return configFileAsString

    def writeLinearizationFunctionToConfigFileString(self, configFileAsString, configClassName):
        configClassString = getConfigClassString(configFileAsString, configClassName)
        linearizeVecPattern = PwmNonlinearityConfigHandler._linearizeVecPattern

        temp = linearizeVecPattern.search(configClassString)
        if temp is not None:
            out = r'\1'
            out += self.getLinearizationFunction(r'\4')
            out += r'\6'
            configClassString = re.sub(linearizeVecPattern, out, configClassString)
            configFileAsString = setConfigClassString(configFileAsString, configClassName, configClassString)

            return configFileAsString

        return ''

    def getLinearizationFunction(self, indent = ''):
        out = ''
        out += indent + 'auto pwmHighFrqCompFun = [](uint16_t in)\n'
        out += indent + '{\n'

        if self.pwmOffset is None:
            lookUpSize = len(self.pwmCompLookUp)
            out += indent + (f'    constexpr static std::array<uint16_t, {lookUpSize}> linearizeVec = '
                            + f'{intArrayToString(self.pwmCompLookUp)}\n')
            out += '\n'
            out += indent +  '    return DefaultConfigHolder::pwmHighFrqCompFun(linearizeVec, in);\n'
        else:
            out += indent +  '    constexpr static uint16_t maxPwm = 1023;\n'
            out += indent + f'    constexpr static uint16_t pwmOffset = {int(round(self.pwmOffset))};\n'
            out += '\n'
            out += indent +  '    if (in == 0)\n'
            out += indent +  '    {\n'
            out += indent +  '        return static_cast<uint16_t>(0);\n'
            out += indent +  '    }\n'
            out += '\n'
            out += indent + ('    return static_cast<uint16_t>(pwmOffset + static_cast<uint32_t>(maxPwm - pwmOffset)'
                                ' * in / maxPwm);\n')

        out += indent + '};'

        return out

class SmoothMoveHandler:
    def __init__(self, startPos, minMoveTime = 0.1):
        self.minMoveTime = minMoveTime
        self.p = startPos
        self.endP = self.p
        self.newEndP = None
        self.maxV = 0.0
        self.w = 0.0
        self.d = 0.0

    def set(self, endP, maxV):
        oldNewEndP = self.newEndP
        oldEndP = self.endP
        if (endP - self.p) * self.d < 0.0:
            self.newEndP = endP
        elif self.newEndP is None:
            oldNewEndP = self.endP
            self.endP = endP

        if endP not in (self.p, oldNewEndP):
            self.maxV = min(maxV, abs(endP - self.p) / self.minMoveTime / 2 * math.pi)

        if oldEndP == self.endP:
            return

        if self.w > math.pi * 0.5:
            self.w = math.pi - self.w

        self.d = (self.endP - self.p) / (1.0 + math.cos(self.w))

    def getNextRef(self, dt):
        if self.w == math.pi:
            self.w = 0.0
            self.d = 0.0
            if self.newEndP is not None:
                temp = self.newEndP
                self.newEndP = None
                self.set(temp, self.maxV)

        eps = 1.0e-100
        if abs(self.d) > eps:
            a = abs(self.maxV / self.d)
        else:
            a = abs(self.maxV / eps)

        self.w = min(self.w + a * dt, math.pi)
        self.p = self.endP - self.d * (1.0 + math.cos(self.w))
        v = self.d * a * math.sin(self.w)
        return self.p, v

class PiecewiseLinearFunction:
    def __init__(self, xList, yList):
        xList = list(xList)
        yList = list(yList)

        if len(xList) != len(yList):
            raise Exception('x and y list not same length')
        if len(xList) < 2:
            raise Exception('x list is too short')

        if sorted(xList) != xList:
            raise Exception('x list is not sorted')

        sortedY = sorted(yList)
        self.monotoneFunction = sortedY in (yList, yList[::-1])

        self.xList = xList
        self.yList = yList

    @staticmethod
    def _findIndex(l, v):
        for i, d in enumerate(l):
            if v < d:
                return i - 1
        return len(l) - 1

    @staticmethod
    def _calcInterpolation(inputList, outputList, inputValue):
        i = PiecewiseLinearFunction._findIndex(inputList, inputValue)
        i = max(i, 0)
        i = min(i, len(inputList) - 2)
        x0 = inputList[i]
        x1 = inputList[i + 1]

        y0 = outputList[i]
        y1 = outputList[i + 1]

        t = (inputValue - x0) / (x1 - x0) if x1 != x0 else 0.5

        return (y1 - y0) * t + y0

    def getX(self, y):
        if not self.monotoneFunction:
            raise Exception('not a monotone function')
        return PiecewiseLinearFunction._calcInterpolation(self.yList, self.xList, y)

    def getY(self, x):
        return PiecewiseLinearFunction._calcInterpolation(self.xList, self.yList, x)
