import ServoProjectModules.Communication as ServoComModule
from ServoProjectModules.Communication import pi

import numba
import numpy as np
import scipy.signal
import threading
import time
import math
import random
import matplotlib.pyplot as plt
from matplotlib.backends.backend_gtk3agg import (
    FigureCanvasGTK3Agg as FigureCanvas)
from matplotlib.figure import Figure
import re
import ServoProjectModules.GuiHelper as GuiFunctions
from ServoProjectModules.GuiHelper import GLib, Gtk

def createServoManager(nodeNr, port, dt=0.004, initFunction=lambda a: a):
    if port != '':
        com = ServoComModule.SerialCommunication(port)
    else:
        com = ServoComModule.SimulateCommunication()

    def createServoFunction(servoManager):
        nonlocal nodeNr
        nonlocal com
        servo = ServoComModule.DCServoCommunicator(nodeNr, com)

        servo.setOffsetAndScaling(2 * pi / 4096.0, 0.950301, 0)

        servo.setControlSpeed(20)
        servo.setBacklashControlSpeed(0, 3.0, 0.00)
        servo.setFrictionCompensation(0)

        servoManager.servoArray.append(servo)

        initFunction(servoManager)


    servoManager = ServoComModule.ServoManager(cycleTime=dt, initFunction=createServoFunction)

    return servoManager

def shrinkArray(a, size, median = False):
    if len(a) <= size:
        return a

    newA = []

    indexScale = len(a) / size

    index = 0

    while index < len(a):
        nextIndex = index + indexScale
        if nextIndex > len(a):
            nextIndex = len(a)

        sortedSubA = sorted(a[int(index): int(nextIndex)])
        if median and len(sortedSubA) > 1:
            i = int(len(sortedSubA) / 2 - 0.49)
            sortedSubA = sortedSubA[i: -i]

        newA.append(sum(sortedSubA) / len(sortedSubA))
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
    classPattern = re.compile(r'(\s*class\s+' + configClassName + r'\s+(.*\n)*?(.*\n)*?\})')
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

wrapAroundAndUnitPerRevPattern = re.compile(r'(?P<beg>return\s+std::make_unique\s*<\s*)(?P<encoderType>\w*)(?P<mid>\s*>\s*\((\w+\s*,\s*))(?P<units>[^;]*)(?P<end>,\s*compVec\s*\)\s*;)')

def getConfiguredOutputEncoderData(configClassString):
    temp = wrapAroundAndUnitPerRevPattern.search(configClassString)
    
    magneticEncoder = temp.group('encoderType') == 'EncoderHandler'
    unitsPerRev = 4096
    if not magneticEncoder:
        unitsStr = wrapAroundAndUnitPerRevPattern.search(configClassString).group('units')

        unitsStr = re.sub(r'f', '', unitsStr)
        unitsPerRev = eval(unitsStr)

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

parmeterPattern = re.compile(r'(?P<beg>.*createMainEncoderHandler\(\)\s*\{(?:.*\n)*?\s*return\s+std::make_unique\s*<\s*(\w*)\s*>\s*\()(?P<params>[^;]*)(?P<end>\s*\)\s*;)')
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
        return str(eval(unitsStr) / 4096)

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
        elif self.newEndP == None:
            oldNewEndP = self.endP
            self.endP = endP

        if endP != self.p and oldNewEndP != endP:
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
            if self.newEndP != None:
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

def main():
    moveHandler = SmoothMoveHandler(0.0, 0.4)
    moveHandler.set(0.0, 0.2)

    refV = 1.0
    r = 1.0
    moveHandler.set(r, refV)

    p, v = moveHandler.getNextRef(0.0)

    pVec = [p]
    vVec = [v]
    rVec = [r]
    for i in range(0, 100):
        p, v = moveHandler.getNextRef(0.01)
        pVec.append(p)
        vVec.append(v)
        rVec.append(r)

    r = 1.5

    for i in range(0, 100):
        moveHandler.set(r, refV)
        p, v = moveHandler.getNextRef(0.01)
        pVec.append(p)
        vVec.append(v)
        rVec.append(r)

    for i in range(0, 500):
        r = 0.5 + min(0.3, i * 0.001)
        moveHandler.set(r, refV)
        p, v = moveHandler.getNextRef(0.01)
        pVec.append(p)
        vVec.append(v)
        rVec.append(r)

    plt.plot(pVec, 'g+')
    plt.plot(vVec, 'y+')
    plt.plot(rVec, 'r+')
    plt.show()

if __name__ == '__main__':
    main()
