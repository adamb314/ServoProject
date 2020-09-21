#!/bin/python3
import numpy as np
import scipy.signal
import math
import matplotlib.pyplot as plt

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

class OpticalEncoderDataVectorGenerator:
    def __init__(self, file):
        data = loadtxtfile(file, (0, 1))

        plt.plot(data[:,0])
        plt.plot(data[:,1])
        plt.show()

        noiseDepresMemLenght = 6
        startIndex = 0
        constVelIndex = 3000
        endIndex = 6000

        a0 = data[0, 0]
        b0 = data[0, 1]
        a1 = data[3, 0]
        b1 = data[3, 1]
        aVec = []
        bVec = []
        for t in np.arange(0.0, 1.0, 1.0 / noiseDepresMemLenght):
            aVec.append(a0 + (a1 - a0) * t)
            bVec.append(b0 + (b1 - b0) * t)

        data = data[constVelIndex:endIndex]
        modData = data
        done = False
        wrapIndex = 0

        def calcCovWithEndOfVectors(aVec, bVec, ca, cb):
            cov = 0

            for (a, b) in zip(aVec[-noiseDepresMemLenght:], bVec[-noiseDepresMemLenght:]):
                cov += (ca - a)**2 + (cb - b)**2

            cov += ((ca - aVec[-noiseDepresMemLenght+1] - (aVec[-1] - aVec[-noiseDepresMemLenght]))**2 + (cb - bVec[-noiseDepresMemLenght+1] - (bVec[-1] - bVec[-noiseDepresMemLenght]))**2)**1.0
            return cov

        while len(modData) > 0:
            print (str(len(modData)) + " data points left to sort...", end="\r")
            minCov = 1000000
            minIndex = 0

            for i, d in enumerate(modData):
                cov = calcCovWithEndOfVectors(aVec, bVec, d[0], d[1])

                if minCov > cov:
                    minIndex = i
                    minCov = cov

            if len(modData) < len(data) * 0.5:
                for i in range(noiseDepresMemLenght, int(len(aVec) / 2)):
                    cov = calcCovWithEndOfVectors(aVec, bVec, aVec[i], bVec[i])

                    if minCov > cov:
                        wrapIndex = i
                        done = True
                        break

            if done:
                break

            aVec.append(modData[minIndex, 0])
            bVec.append(modData[minIndex, 1])

            modData = np.delete(modData, minIndex, 0)

        aVec = aVec[wrapIndex:-1]
        bVec = bVec[wrapIndex:-1]

        aVecShrunk = shrinkArray(aVec, 512)
        bVecShrunk = shrinkArray(bVec, 512)

        self.aVec = aVec
        self.bVec = bVec
        self.aVecShrunk = aVecShrunk
        self.bVecShrunk = bVecShrunk

    def plotGeneratedVectors(self):
        x = np.arange(len(self.aVec)) * len(self.aVecShrunk) / len(self.aVec)
        plt.plot(x, self.aVec, 'r+')
        plt.plot(x, self.bVec, 'g+')

        plt.plot(self.aVecShrunk, 'r.-')
        plt.plot(self.bVecShrunk, 'g.-')
        plt.show()

def sign(v):
    if v >= 0:
        return 1.0
    return -1.0

class SystemIdentificationObject:
    def __init__(self, file):
        data = loadtxtfile(file, (1, 3, 5, 7))

        self.dt = data[1, 0] - data[0, 0]

        tempVelData = 0 * data[:, 1]
        for i, d in enumerate(zip(data[2:,1], data[0:-2,1])):
            tempVelData[i + 1] = (d[0] - d[1]) / self.dt

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
        self.currentModelParams = np.array([1.0, self.servoModelParameters[2,0]])
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
                phi = np.matrix([[d[1][0]], [d[2][0]], [d[1][0] * d[2][0]], [1.0]])
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
        self.servoModelParameters[2,0] = self.servoModelParameters[2,0] / self.servoModelParameters[1,0]
        self.servoModelParameters[3,0] = -self.servoModelParameters[3,0] / self.servoModelParameters[1,0]
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
                    friction = -self.servoModelParameters[3,0]
                elif pwm < 0:
                    friction = self.servoModelParameters[3,0]
                simVel[i] = (self.servoModelParameters[0] * lastSimVel +
                    self.servoModelParameters[1] * (pwm + self.servoModelParameters[2] * lastSimVel * abs(pwm) + friction))
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

            self.kalmanFilter = KalmanFilter(dt, self.A, self.B, self.C, 60 * 4 * 2)

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
        opticalEncoderDataVectorGenerator = OpticalEncoderDataVectorGenerator(rawDataVecotrsFile)
        
        if args.plotData:
            opticalEncoderDataVectorGenerator.plotGeneratedVectors()
        
        out += "    static std::unique_ptr<OpticalEncoderHandler> createMainEncoderHandler()\n"
        out += "    {\n"
        out += "        std::array<uint16_t, 512> aVec = " + intArrayToString(opticalEncoderDataVectorGenerator.aVecShrunk) + "\n"
        out += "        std::array<uint16_t, 512> bVec = " + intArrayToString(opticalEncoderDataVectorGenerator.bVecShrunk) + "\n"
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
        systemParams = systemIdentifier.getServoSystemModelParameters(0.0012)

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
        out += "            return " + str(systemIdentifier.servoModelParameters[3,0]) + ";\n"
        out += "        }\n"
        out += "\n"
        out += "        //state feedback vecktor\n"
        out += "        static Eigen::Matrix<float, 4, 1> getLVector(uint8_t controllerSpeed)\n"
        out += "        {\n"
        out += "            float dt = getAMatrix()(0, 1);\n"
        out += "            float a = getAMatrix()(1, 1);\n"
        out += "            float b = getBVector()(1);\n"
        out += "\n"
        out += "            return calculateLVector(controllerSpeed, dt, a, b);\n"
        out += "        }\n"
        out += "    };\n"

    out += "};\n"
    out += "// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"
    print(out)

if __name__ == "__main__":
    main();