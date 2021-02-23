#!/bin/python3
import numpy as np
import scipy.signal
import math
import matplotlib.pyplot as plt
import numba

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
    def __init__(self, file, segment = 3000,
            constVelIndex = 10000, noiseDepresMemLenght = 5):

        self.data = loadtxtfile(file, (0, 1))
        self.noiseDepresMemLenght = noiseDepresMemLenght

        plt.plot(self.data[:,0], 'r')
        plt.plot(self.data[:,1], 'g')
        plt.show()

        maxA = 0
        maxAIndex = 0
        a0 = self.data[0, 0]
        armed = False

        for i, d in enumerate(self.data[0:1000, 0]):
            if abs(a0 - d) > 150:
                armed = True

            if armed and abs(a0 - d) < 50:
                startIndex = i
                break

        print(startIndex)

        self.a0 = self.data[startIndex, 0]
        self.b0 = self.data[startIndex, 1]
        self.a1 = self.data[startIndex + noiseDepresMemLenght, 0]
        self.b1 = self.data[startIndex + noiseDepresMemLenght, 1]

        self.aVecList = []
        self.bVecList = []

        self.aVec = np.zeros(512)
        self.bVec = np.zeros(512)

        nr = int((len(self.data) - constVelIndex) / segment)
        actNr = 0
        for i in range(0, nr):
            print(str(i + 1) + ' of ' + str(nr))
            try:
                aVecShrunk, bVecShrunk, aVec, bVec = self.genVec(constVelIndex + segment * i, constVelIndex + segment * (i + 1))
                self.aVec += aVecShrunk
                self.bVec += bVecShrunk

                self.aVecList.append(aVec)
                self.bVecList.append(bVec)

                actNr += 1

            except:
                pass

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

        while len(modData) > 0:
            print (str(len(modData)) + " data points left to sort...      ", end="\r")
            
            minIndex, done, wrapIndex = findBestFitt(data, modData, aVec, bVec, self.noiseDepresMemLenght)

            if done:
                break

            aVec.append(modData[minIndex, 0])
            bVec.append(modData[minIndex, 1])

            modData = np.delete(modData, minIndex, 0)

        aVec = aVec[wrapIndex:]
        bVec = bVec[wrapIndex:]

        while len(modData) > 0:
            print (str(len(modData)) + " data points left to sort...      ", end="\r")
            
            minIndex = findBestFitt2(modData[0, 0], modData[0, 1], aVec, bVec)

            aVec.insert(minIndex, modData[0, 0])
            bVec.insert(minIndex, modData[0, 1])

            modData = np.delete(modData, 0, 0)

        print ("                                 ", end="\r")
        print ("data points sorted")

        if abs(aVec[0] - self.a0) > 20:
                raise Exception("Too ruff")

        if abs(bVec[0] - self.b0) > 20:
                raise Exception("Too ruff")

        for d in zip(aVec[0:-1], aVec[1:]):
            if abs(d[1] - d[0]) > 20:
                raise Exception("Too ruff")

        for d in zip(bVec[0:-1], bVec[1:]):
            if abs(d[1] - d[0]) > 20:
                raise Exception("Too ruff")

        dirNoiseDepresMemLenght = int(len(data) / 50)

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

    def plotGeneratedVectors(self):
        plt.plot(self.data[:,0], 'r')
        plt.plot(self.data[:,1], 'g')
        plt.show()

        for d in zip(self.aVecList, self.bVecList):
            x = np.arange(len(d[0])) * len(self.aVec) / len(d[0])
            plt.plot(x, d[0], '-')
            plt.plot(x, d[1], '-')

        plt.plot(self.aVec, 'r.-')
        plt.plot(self.bVec, 'g.-')

        plt.show()

def sign(v):
    if v >= 0:
        return 1.0
    return -1.0

class SystemIdentificationObject:
    def __init__(self, file):
        data = loadtxtfile(file, (1, 3, 5, 7))

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
            if lastVel == None:
                lastVel = d
            if abs(d - lastVel) < 300000:
                data[i, 1] = d
                lastVel = d
            else:
                data[i, 1] = lastVel
                lastVel = minDiff(tempVelData[i - 5:i], lastVel)

        data = data[1:-1]

        plt.plot(data[:, 1])
        plt.plot(data[:, 2])
        plt.plot(data[:, 3])
        plt.show()

        velData = data[:, 1]
        pwmData = data[:, 3]

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


    def plotDataVector(self):
        t = np.arange(len(self.pwmData)) * self.dt
        plt.figure(1)
        plt.plot(t, self.pwmData)
        plt.figure(2)
        plt.plot(t, self.velData)
        plt.show()

    def identifyCurrentSystemModel(self):
        self.currentModelParams = np.array([1.0, self.servoModelParameters[3,0]])
        return self.currentModelParams

    def plotCurrentSystemModel(self):
        print("currentModelParams = " + str(self.currentModelParams))
        simCurrent = self.currentModelParams[0] * self.pwmData + self.currentModelParams[1] * abs(self.pwmData) * self.velData

        t = np.arange(len(self.pwmData)) * self.dt
        plt.plot(t, simCurrent, 'k')
        plt.show()

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

        print("cov = " + str(cov))
        print("covY = " + str(covY))
        self.servoModelParameters = np.linalg.solve(cov, covY)
        self.servoModelParameters[2,0] = -self.servoModelParameters[2,0] / self.servoModelParameters[1,0]
        self.servoModelParameters[3,0] = self.servoModelParameters[3,0] / self.servoModelParameters[1,0]
        print("servoModelParameters = " + str(self.servoModelParameters))
        return self.servoModelParameters

    def plotServoSystemModel(self):
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

        plt.figure(1)
        t = np.arange(len(simVel)) * self.dt
        plt.plot(t, self.velData)
        plt.plot(t, simVel, 'k')

        plt.show()

class KalmanFilter(object):
    """docstring for KalmanFilter"""
    def __init__(self, dt, A, B, C, kalmanFilterSpeed):
        super(KalmanFilter, self).__init__()

        Aex = np.hstack((A, B))
        Aex = np.vstack((Aex, np.array([[0, 0, 1]])))

        Bex = np.vstack((B, np.array([[0]])))
        Cex = np.hstack((C, np.array([[0]])))

        xhatex = np.zeros(np.shape(Bex))

        poles = np.exp(dt * np.array([-1.0, -0.98, -0.96]) * kalmanFilterSpeed)

        self.A = Aex
        self.AInv = np.linalg.inv(Aex)
        self.B = Bex
        self.C = Cex

        AT = np.transpose(Aex)
        CT = np.transpose(Cex)

        plaseResult = scipy.signal.place_poles(AT,  CT, poles)

        self.K = np.transpose(plaseResult.gain_matrix)

    def printMatrices(self):
        print("A << " + printAsEigenInit(self.A))
        print("AInv << " + printAsEigenInit(self.AInv))
        print("B << " + printAsEigenInit(self.B))
        print("C << " + printAsEigenInit(self.C))
        print("K << " + printAsEigenInit(self.K))

class ServoModel(object):
    """docstring for ServoModel"""
    def __init__(self, dt, a, b):
            super(ServoModel, self).__init__()
            self.dt = dt
            dtp = self.dt
            dt2p = self.dt**2
            self.A = np.array([[1, dtp], [0, a]])
            self.B = np.array([[dt2p / 2], [dtp]]) * b
            self.C = np.array([[1, 0]])

            self.kalmanFilter = KalmanFilter(dt, self.A, self.B, self.C, 30 * 4 * 4)

import sys
import argparse

def main():
    parser = argparse.ArgumentParser(description='Script to analyze servo data recordings and generate config data')
    parser.add_argument('--opticalEncoderDataFile', nargs='?', type=argparse.FileType('r'),
                        help='optical encoder data file to load')

    parser.add_argument('--systemIdentDataFile', nargs='?', type=argparse.FileType('r'),
                        help='system identification data file to load')

    parser.add_argument('-p', '--plotData', type=bool, default=False,
                        help='plot the recorded data (default is False)')

    args = parser.parse_args()

    out = ""

    if args.opticalEncoderDataFile == None and args.systemIdentDataFile == None:
        out += "// replace with generated output from systemIdent.py\n"
        out += "// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv\n"
        out += "// config setps\n"
        out += "// 1) disconnect motor from gearbox\n"
        out += "// 2) compile and transfer to servo nr x\n"
        out += "// 3) open MasterCommunication folder in terminal\n"
        out += "// 4) run 'make'\n"
        out += "// 5) run './executable --servoNr x --recOpticalEncoder --output=opticalEncoderData.txt'\n"
        out += "// 6) run './systemIdent.py --opticalEncoderDataFile=opticalEncoderData.txt'\n"
        out += "// 7) copy past new generated ConfigHolder class, from terminal, over old class\n"
        out += "// 8) compile and transfer to servo nr x with new ConfigHolder class\n"
        out += "// 9) run './executable --servoNr x --recSystemIdentData --output=systemIdentData.txt'\n"
        out += "// 10) run './systemIdent.py --opticalEncoderDataFile=opticalEncoderData.txt --systemIdentDataFile=systemIdentData.txt'\n"
        out += "// 11) copy past new generated ConfigHolder class, from terminal, over old class\n"
        out += "// 12) connect motor to gearbox again\n"
        out += "// 13) compile and transfer to servo nr x with new ConfigHolder class\n"

    elif args.systemIdentDataFile == None:
        out += "// replace with generated output from systemIdent.py\n"
        out += "// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv\n"
        out += "// config setps\n"
        out += "// x 1) disconnect motor from gearbox\n"
        out += "// x 2) compile and transfer to servo nr x\n"
        out += "// x 3) open MasterCommunication folder in terminal\n"
        out += "// x 4) run 'make'\n"
        out += "// x 5) run './executable --servoNr x --recOpticalEncoder --output=opticalEncoderData.txt'\n"
        out += "// x 6) run './systemIdent.py --opticalEncoderDataFile=opticalEncoderData.txt'\n"
        out += "// x 7) copy past new generated ConfigHolder class, from terminal, over old class\n"
        out += "// 8) compile and transfer to servo nr x with new ConfigHolder class\n"
        out += "// 9) run './executable --servoNr x --recSystemIdentData --output=systemIdentData.txt'\n"
        out += "// 10) run './systemIdent.py --opticalEncoderDataFile=opticalEncoderData.txt --systemIdentDataFile=systemIdentData.txt'\n"
        out += "// 11) copy past new generated ConfigHolder class, from terminal, over old class\n"
        out += "// 12) connect motor to gearbox again\n"
        out += "// 13) compile and transfer to servo nr x with new ConfigHolder class\n"

    else:
        out += "// replace with generated output from systemIdent.py\n"
        out += "// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv\n"
        out += "// config setps\n"
        out += "// x 1) disconnect motor from gearbox\n"
        out += "// x 2) compile and transfer to servo nr x\n"
        out += "// x 3) open MasterCommunication folder in terminal\n"
        out += "// x 4) run 'make'\n"
        out += "// x 5) run './executable --servoNr x --recOpticalEncoder --output=opticalEncoderData.txt'\n"
        out += "// x 6) run './systemIdent.py --opticalEncoderDataFile=opticalEncoderData.txt'\n"
        out += "// x 7) copy past new generated ConfigHolder class, from terminal, over old class\n"
        out += "// x 8) compile and transfer to servo nr x with new ConfigHolder class\n"
        out += "// x 9) run './executable --servoNr x --recSystemIdentData --output=systemIdentData.txt'\n"
        out += "// x 10) run './systemIdent.py --opticalEncoderDataFile=opticalEncoderData.txt --systemIdentDataFile=systemIdentData.txt'\n"
        out += "// x 11) copy past new generated ConfigHolder class, from terminal, over old class\n"
        out += "// 12) connect motor to gearbox again\n"
        out += "// 13) compile and transfer to servo nr x with new ConfigHolder class\n"

    out += "class ConfigHolder : public SetupConfigHolder\n"
    out += "{\n"
    out += "public:\n"

    rawDataVecotrsFile = args.opticalEncoderDataFile
    if rawDataVecotrsFile == None:
        pass
    else:
        
        opticalEncoderDataVectorGenerator = OpticalEncoderDataVectorGenerator(
                rawDataVecotrsFile, constVelIndex=10000, noiseDepresMemLenght=8)

        if args.plotData:
            opticalEncoderDataVectorGenerator.plotGeneratedVectors()

        out += "    static std::unique_ptr<OpticalEncoderHandler> createMainEncoderHandler()\n"
        out += "    {\n"
        out += "        std::array<uint16_t, 512> aVec = " + intArrayToString(opticalEncoderDataVectorGenerator.aVec) + "\n"
        out += "        std::array<uint16_t, 512> bVec = " + intArrayToString(opticalEncoderDataVectorGenerator.bVec) + "\n"
        out += "        return std::make_unique<OpticalEncoderHandler>(aVec, bVec);\n"
        out += "    }\n"
        out += "\n"
        
    dataVecotorsFile = args.systemIdentDataFile
    if dataVecotorsFile == None:
        out += "    class ControlParameters : public SetupConfigHolder::DefaultControlParameters\n"
        out += "    {\n"
        out += "      public:\n"
        out += "    };\n"

    else:
        systemIdentifier = SystemIdentificationObject(dataVecotorsFile)

        if args.plotData:
            systemIdentifier.plotDataVector()
            systemIdentifier.plotServoSystemModel()
            systemIdentifier.plotCurrentSystemModel()

        dt = 0.0012
        systemParams = systemIdentifier.getServoSystemModelParameters(dt)

        servoModel = ServoModel(dt, systemParams[0], systemParams[1])

        out += "    static std::unique_ptr<CurrentController> createCurrentController()\n"
        out += "    {\n"
        out += "        constexpr float pwmToStallCurrent{" + str(systemIdentifier.currentModelParams[0]) + "};\n"
        out += "        constexpr float backEmfCurrent{" + str(systemIdentifier.currentModelParams[1]) + "};\n"
        out += "\n"
        out += "        return std::make_unique<CurrentControlModel>(pwmToStallCurrent, backEmfCurrent);\n"
        out += "    }\n"
        out += "\n"
        out += "    class ControlParameters : public SetupConfigHolder::DefaultControlParameters\n"
        out += "    {\n"
        out += "      public:\n"
        out += "        //kalman filter observer vector\n"
        out += "        static Eigen::Vector3f getKVector()\n"
        out += "        {\n"
        out += "            Eigen::Vector3f K;\n"
        out += "            K << " + printAsEigenInit(servoModel.kalmanFilter.K, "                ")
        out += "\n"
        out += "            return K;\n"
        out += "        }\n"
        out += "\n"
        out += "        //system model A matrix\n"
        out += "        static Eigen::Matrix3f getAMatrix()\n"
        out += "        {\n"
        out += "            Eigen::Matrix3f A;\n"
        out += "            A << " + printAsEigenInit(servoModel.kalmanFilter.A, "                ")
        out += "\n"
        out += "            return A;\n"
        out += "        }\n"
        out += "\n"
        out += "        //system model invers A matrix\n"
        out += "        static Eigen::Matrix3f getAInvMatrix()\n"
        out += "        {\n"
        out += "            Eigen::Matrix3f AInv;\n"
        out += "            AInv << " + printAsEigenInit(servoModel.kalmanFilter.AInv, "                ")
        out += "\n"
        out += "            return AInv;\n"
        out += "        }\n"
        out += "\n"
        out += "        //system model B matrix\n"
        out += "        static Eigen::Vector3f getBVector()\n"
        out += "        {\n"
        out += "            Eigen::Vector3f B;\n"
        out += "            B << " + printAsEigenInit(servoModel.kalmanFilter.B, "                ")
        out += "\n"
        out += "            return B;\n"
        out += "        }\n"
        out += "\n"
        out += "        //system model friction comp value\n"
        out += "        static float getFrictionComp()\n"
        out += "        {\n"
        out += "            return " + str(systemIdentifier.servoModelParameters[2,0]) + ";\n"
        out += "        }\n"
        out += "    };\n"

    out += "};\n"
    out += "// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"
    print(out)

if __name__ == "__main__":
    main();