'''
Module for calibrating pwm nonlinearity
'''
# pylint: disable=duplicate-code, too-many-lines

import os
import warnings
from ServoProjectModules.CalibrationAnalyzers.Helper import *  # pylint: disable=wildcard-import, unused-wildcard-import

def discToContA(a, dt):
    #discreteA = math.exp(-contineusA * dt) =>
    return -math.log(a) / dt

def discToContB(a, b, dt):
    #discreteB = contineusB * (1.0 - discreteA) / contineusA =>
    return b * discToContA(a, dt) / (1.0 - a)

def highPassFilter(v, a):
    v = np.array(v)
    out = 0 * v + v[0]
    for i in range(0, len(v) - 1):
        out[i + 1] = a * out[i] + a * (v[i + 1] - v[i])
    return out

def lowPassFilter(v, a):
    v = np.array(v)
    out = 0 * v + v[0]
    for i in range(0, len(v) - 1):
        out[i + 1] = a * out[i] + (1.0 - a) * v[i + 1]
    return out

def calcMedian(vec):
    vec = sorted(vec)
    out = vec[len(vec) // 2 - 1]
    if len(vec) % 2 == 0:
        out += vec[len(vec) // 2]
        out = out / 2
    return out

def fitPolynome(x, y, degree):
    p = np.polyfit(x, y, degree)
    return p

def renderPolynome(x, p):
    y = np.polyval(p, x)
    return y

def getMinPolynomeRepFrom(xList, p):
    t = [v / (len(p) - 1) for v in range(0, len(p))]
    sparceX = sorted(xList)
    sparceX = [sparceX[0] + (sparceX[-1] - sparceX[0]) * v for v in t]
    out = [renderPolynome(x, p) for x in sparceX]
    return (sparceX, out)

def getPolynomeFrom(polynomeRep):
    return fitPolynome(polynomeRep[0], polynomeRep[1], len(polynomeRep[1]) - 1)

def findPwmChangeDelay(pwmData):
    lastPwm = pwmData[2]
    constPwmSamples = []
    nrOfConstPwm = 0
    for pwm in pwmData:
        if lastPwm == pwm:
            nrOfConstPwm += 1
        else:
            constPwmSamples.append(nrOfConstPwm)
            nrOfConstPwm = 0
        lastPwm = pwm

    constPwmSamples = sorted(constPwmSamples)
    return constPwmSamples[len(constPwmSamples) // 20]

def calcVelFromPos(posData, timeData, dStep):
    velData = np.array([0.0 for _ in range(0, len(posData))])
    for i, _ in enumerate(zip(posData[dStep:], posData[0:-dStep])):
        velData[i + dStep] = (posData[i+dStep] - posData[i]) / (timeData[i+dStep] - timeData[i])

    return velData

def calcParamsAndPwmLists(pwmData, velData, dStep, calcPhiAndY, *, nrOfPwmAmpsToCombine=1,
        skipSampleIf=lambda d: False):
    # pylint: disable=too-many-locals, too-many-statements
    index = -1
    tempPwmList = []
    phi2SumList = []
    yPhiSumList = []
    phiSumList = []
    ySumList = []
    nrOfSumList = []

    pltVec = []

    lastPwm = math.nan
    for i, d in enumerate(zip(velData[dStep:], velData[0:-dStep], pwmData[0:-dStep])):
        pwmSlice = pwmData[max(i - dStep - 1, 0):i + dStep]
        constPwm = all(pwmSlice[0] == d for d in pwmSlice[1:])
        constPwm = constPwm and not math.isnan(pwmSlice[0])
        if constPwm and not skipSampleIf(d):
            if abs(lastPwm) != abs(d[2]):
                index += 1
                tempPwmList.append(abs(d[2]))
            lastPwm = d[2]

            pltVec.append(d[0])

            phi, y = calcPhiAndY(d)

            phi2 = phi * np.transpose(phi)
            yPhi = y * phi
            if index == len(nrOfSumList):
                phi2SumList.append(phi2)
                yPhiSumList.append(yPhi)
                phiSumList.append(phi)
                ySumList.append(y)
                nrOfSumList.append(1)
            else:
                phi2SumList[index] += phi2
                yPhiSumList[index] += yPhi
                phiSumList[index] += phi
                ySumList[index] += y
                nrOfSumList[index] += 1

    def sumAdjacent(l, nr):
        out = []
        for i in range(0, len(l) - nr + 1):
            out.append(sum(l[i:i + nr]))
        return out

    def avarageAdjacent(l, nr):
        return [v / nr for v in sumAdjacent(l, nr)]

    if nrOfPwmAmpsToCombine <= 0:
        nrOfPwmAmpsToCombine = len(phi2SumList)

    phi2SumList = sumAdjacent(phi2SumList, nrOfPwmAmpsToCombine)
    yPhiSumList = sumAdjacent(yPhiSumList, nrOfPwmAmpsToCombine)
    phiSumList = sumAdjacent(phiSumList, nrOfPwmAmpsToCombine)
    ySumList = sumAdjacent(ySumList, nrOfPwmAmpsToCombine)
    nrOfSumList = sumAdjacent(nrOfSumList, nrOfPwmAmpsToCombine)
    tempPwmList = avarageAdjacent(tempPwmList, nrOfPwmAmpsToCombine)

    phiCovList = []
    yPhiCovList = []
    for phi2Sum, yPhiSum, phiSum, ySum, nr in zip(phi2SumList, yPhiSumList, phiSumList, ySumList, nrOfSumList):
        phiCovList.append(phi2Sum * nr - phiSum * np.transpose(phiSum))
        yPhiCovList.append(yPhiSum * nr - phiSum * np.transpose(ySum))

    pwmList = []
    paramsList = []

    for d in zip(phiCovList, yPhiCovList, tempPwmList, nrOfSumList):
        try:
            if d[3] < 50:
                continue
            params = np.linalg.solve(d[0], d[1])
            pwmList.append(d[2])
            paramsList.append(params)
        except Exception:
            pass

    return (pwmList, paramsList)

class SystemIdentificationObject:
    # pylint: disable=too-many-instance-attributes
    def __init__(self, data, *, additionalData=None, ad=None, bd=None, friction=None, pwmOffset=None, dt=None,
                shouldAbort=lambda:False, updateProgress=lambda v:None):
        # pylint: disable=too-many-locals, too-many-statements

        if additionalData is None:
            additionalData = []

        if len(data) == 0 and len(additionalData) == 0:
            self.dt = dt
            self.velData = np.array([[0], [0]])
            self.pwmData = np.array([[0], [0]])

            self.servoModelParameters = np.array([ad, bd, friction, pwmOffset])
            return

        def extractDataVectors(data):
            dt = np.mean(data[1:, 0] - data[0:-1, 0])
            t = np.array(data[:, 0])
            wallTime = np.array(data[:, 3])
            posData = data[:, 1]
            pwmData = np.array(data[:, 2])

            dtError = wallTime - t
            x = np.arange(0, len(dtError))
            dtErrorPoly = np.polyfit(x, dtError, 4)
            dtError -= np.polyval(dtErrorPoly, x)
            outOfDtSync = [abs(d / dt) >= 0.5 for d in dtError]

            dStep = 2

            for i, d in enumerate(outOfDtSync):
                if d:
                    pwmData[i] = math.nan
                    if i != 0:
                        pwmData[i - 1] = math.nan
                    if i != len(pwmData) - 1:
                        pwmData[i + 1] = math.nan

            velData = calcVelFromPos(posData, wallTime, dStep)
            velData = velData[dStep:]
            pwmData = pwmData[dStep:]

            return (dt, dStep, dtError, pwmData, velData)

        def extractParametersVsPwm(dt, dStep, pwmData, velData):
            dt = dt * dStep

            def calcPhiAndY(d):
                phi = np.array([[d[1]], [d[2]]])
                y = d[0]

                return (phi, y)

            pwmList, paramsList = calcParamsAndPwmLists(
                    pwmData, velData, dStep, calcPhiAndY,
                    nrOfPwmAmpsToCombine=1, skipSampleIf=
                    lambda d: sign(d[2]) != sign(d[1])
                    )

            pwmList = pwmList[::-1]
            aList = [d[0, 0] for d in paramsList[::-1]]
            bList = [d[1, 0] for d in paramsList[::-1]]

            bContList = [discToContB(a, b, dt) for a, b in zip(aList, bList)]
            aContList = [discToContA(a, dt) for a in aList]

            return (pwmList, aContList, bContList, aList, bList)

        tempListForSorting = []
        self.dtError = []

        dataLists = []
        if len(additionalData) != 0:
            dataLists += additionalData[1:] + [additionalData[0]]
        dataLists += [data]
        for d in dataLists:
            if len(d) == 0:
                continue

            self.dt, dStep, dtError, self.pwmData, self.velData = extractDataVectors(d)

            self.dtError += list(dtError)
            self.pwmChangeDelay = findPwmChangeDelay(d[:, 2])

            pwmList, aList, bList, self.aDiscList, self.bDiscList = extractParametersVsPwm(
                    self.dt, dStep, self.pwmData, self.velData)

            tempListForSorting += list(zip(pwmList, aList, bList))

        tempListForSorting.sort(key = lambda d: d[0])
        pwmList = [d[0] for d in tempListForSorting]
        aList = [d[1] for d in tempListForSorting]
        bList = [d[2] for d in tempListForSorting]

        def calcModelParameters(pwmList, aList, bList, nrOfFricOptStep=10):
            # pylint: disable=too-many-statements
            testData = []
            if len(data) != 0:
                testData += [data]
            testData += additionalData

            def getErrorAndParamsForFrictionComp(uFric):
                def getPwmCompPolynome(pwm, comp, degree):
                    interpolSample = 1
                    extrapolate = 100

                    linearInterpol = PiecewiseLinearFunction(pwm, comp)
                    pwmInter = np.arange(0, (pwm[-1] - pwm[0]) // interpolSample)
                    pwmInter = (pwm[-1] - pwm[0]) / (pwmInter[-1] - pwmInter[0]) * pwmInter + pwm[0]
                    pwmInter = list(pwmInter)
                    compInter = [linearInterpol.getY(v) for v in pwmInter]

                    xn = comp[-1]
                    k = (comp[-1] - comp[0]) / (pwm[-1] - pwm[0])
                    k = max(k, xn / pwm[-1])

                    xPad = np.arange(pwm[-1] + interpolSample, 1023 + extrapolate, interpolSample)
                    compPad = xn + k * (xPad - pwm[-1])

                    xAtZero = -xn / k + pwm[-1]
                    xPad0 = np.arange(-extrapolate, xAtZero, interpolSample)

                    compPad0 = xn + k * (xPad0 - pwm[-1])
                    pwmExt = list(xPad0) + pwmInter + list(xPad)
                    compExt = list(compPad0) + compInter + list(compPad)

                    warnings.simplefilter('ignore', np.RankWarning)

                    return fitPolynome(pwmExt, compExt, degree), (
                            (pwm, comp), (pwmInter, compInter), (pwmExt, compExt))

                pwmCompList = [u * b + uFric for u, b in zip(pwmList, bList)]
                p, pwmCompDiagData = getPwmCompPolynome(pwmList, pwmCompList, 32)
                contineusB = max(renderPolynome(np.arange(0, 1024, 1), p)) / 1023

                p = p / contineusB
                friction = uFric / contineusB

                pwmCompModel = getMinPolynomeRepFrom([0.0, 1023.0], p)

                pwmCompFun = SystemIdentificationObject.getPwmLinearizer(pwmCompModel)
                linearPwmList = [pwmCompFun.getY(x) for x in pwmList]

                p = np.polyfit(linearPwmList, aList, 1)

                contineusA = p[1]
                emf = p[0]
                if emf < 0:
                    emf = 0
                    contineusA = median(aList)

                modelParameters = (contineusA, contineusB, friction, emf, pwmCompModel)
                _, _, _, errorSum = self.calcSimResults(testData[0], modelParameters,
                        pwmList=pwmList, aList=aList)

                return errorSum, modelParameters, pwmCompDiagData

            uFric0 = 0
            uFric1 = bList[-1] * pwmList[0]

            lastFriction = 0
            for i in range(1, max(2, nrOfFricOptStep+1)):
                updateProgress(i / (nrOfFricOptStep+1))

                e0, modelParameters, pwmCompDiagData = getErrorAndParamsForFrictionComp(uFric0)

                if nrOfFricOptStep == 0:
                    return modelParameters, pwmCompDiagData

                e1, _, _ = getErrorAndParamsForFrictionComp(uFric1)

                if shouldAbort():
                    return modelParameters, pwmCompDiagData

                uFric = -e0 / (e1 - e0) * (uFric1 - uFric0) + uFric0
                uFric = max(0.0, uFric)

                e, modelParameters, pwmCompDiagData = getErrorAndParamsForFrictionComp(uFric)

                if int(round(modelParameters[2])) == int(round(lastFriction)) or uFric == 0.0:
                    break
                lastFriction = modelParameters[2]

                if e < 0:
                    uFric1 = uFric
                else:
                    uFric0 = uFric

            updateProgress(1.0)

            return modelParameters, pwmCompDiagData

        self.servoModelParameters, self.pwmCompDiagData = calcModelParameters(pwmList, aList, bList, nrOfFricOptStep=10)

        self.pwmList = pwmList
        self.aList = aList
        self.pwmCompList = [u * b / self.servoModelParameters[1] + self.servoModelParameters[2]
                            for u, b in zip(pwmList, bList)]

        self.simVel = []
        self.simRefVel = []
        self.simError = []

        for d in [data] + additionalData:
            if len(d) == 0:
                continue
            tempSimVel, tempSimRefVel, tempSimError, errorSum = self.calcSimResults(d,
                self.servoModelParameters)

            print(f'{errorSum = }')

            self.simVel += list(tempSimVel)
            self.simRefVel += list(tempSimRefVel)
            self.simError += list(tempSimError)

    @staticmethod
    def checkForPreviousCalibration(configFileAsString, configClassName):
        return PwmNonlinearityConfigHandler.checkForPreviousCalibration(configFileAsString, configClassName)

    @staticmethod
    def getPwmLinearizer(pwmCompModel):
        p = getPolynomeFrom(pwmCompModel)
        x = np.array(pwmCompModel[0])
        y = renderPolynome(x, p)

        fun = PiecewiseLinearFunction(x, y)
        xAtZero = max(0.01, fun.getX(0, takeLowest=True))

        x = np.arange(xAtZero, 1024, (1024 - xAtZero) / len(x))
        y = renderPolynome(x, p)
        y = np.array([max(0.01, v) for v in y])

        x = list(-x[::-1]) + list(x)
        y = list(-y[::-1]) + list(y)

        return PiecewiseLinearFunction(x, y)

    def getServoSystemMatrices(self, outputDt):
        contineusA = self.servoModelParameters[0]
        contineusB = self.servoModelParameters[1]

        scaledDiscreteA = math.exp(outputDt * -contineusA)
        scaledDiscreteB = contineusB * (1.0 - scaledDiscreteA) / contineusA
        scaledDiscretePosA = (1.0 - scaledDiscreteA) / contineusA
        scaledDiscretePosB = contineusB * (outputDt - scaledDiscretePosA) / contineusA

        aMat = np.array([[1.0, scaledDiscretePosA], [0.0, scaledDiscreteA]])
        bMat = np.array([[scaledDiscretePosB], [scaledDiscreteB]])

        return (aMat, bMat)

    def getServoSystemModelParameters(self, outputDt):
        aMat, bMat = self.getServoSystemMatrices(outputDt)

        pwmCompModel = self.servoModelParameters[4]
        pwmCompFun = SystemIdentificationObject.getPwmLinearizer(pwmCompModel)

        pwmList = [i * 1023 / 255 for i in range(0, 256)]
        linearPwmList = [max(0, pwmCompFun.getX(v, takeLowest=True)) for v in pwmList]
        linearPwmList[0] = 0

        return (aMat[1, 1], bMat[1, 0], self.servoModelParameters[2], (pwmList, linearPwmList))

    def getCurrentModelParameters(self):
        _, contineusB, _, emf, _ = self.servoModelParameters

        # dy / dt = -(a + emf * abs(pwm)) * y + b * pwm =
        #         = -a * y + b * (pwm - emf / b * y * abs(pwm)) =>
        currentModelParams = [1.0, -emf / contineusB]
        return currentModelParams

    @staticmethod
    def calcSimResults(data, modelParameters, *, pwmList=None, aList=None):
        # pylint: disable=too-many-locals, too-many-statements

        contineusA, contineusB, friction, emf, pwmCompModel = modelParameters
        pwmCompFun = SystemIdentificationObject.getPwmLinearizer(pwmCompModel)

        dt = data[1, 0] - data[0, 0]
        pwmChangeDelay = findPwmChangeDelay(data[:, 2])

        realTime = data[2:, 3]
        realPos = data[2:, 1]
        pwmData = data[2:, 2]

        linearPwmData = [pwmCompFun.getY(pwm) for pwm in pwmData]

        aDict ={round(pwmCompFun.getY(abs(p))): contineusA + emf * pwmCompFun.getY(abs(p)) for p in pwmData}
        if pwmList is not None:
            aDict = {round(pwmCompFun.getY(abs(p))): v for p, v in zip(pwmList, aList)}

        realVel = calcVelFromPos(realPos, realTime, 1)

        simVel = 0 * realPos + realVel[0]
        simPos = 0 * realPos + realPos[0]
        lastSimVel = simVel[0]
        lastSimPos = simPos[0]

        for i, linearPwm in enumerate(linearPwmData[0:-3]):
            aTemp = aDict[round(abs(linearPwm))]
            adTemp = math.exp(dt * -aTemp)
            bdTemp = contineusB * (1.0 - adTemp) / aTemp

            pwmSlice = linearPwmData[max(i - 2, 0):i + 2]
            constPwm = all(pwmSlice[0] == d for d in pwmSlice[1:])

            if constPwm:
                newVel = (adTemp * lastSimVel +
                    bdTemp * (linearPwm - friction * sign(lastSimVel)))

                if sign(newVel) != sign(lastSimVel):
                    subSetp = 10
                    for _ in range(0, subSetp):
                        aTemp = aDict[round(abs(linearPwm))]
                        adTemp = math.exp((dt / subSetp) * -aTemp)
                        bdTemp = contineusB * (1.0 - adTemp) / aTemp

                        newVel = (adTemp * lastSimVel +
                            bdTemp * (linearPwm - friction * sign(lastSimVel)))

                        lastSimPos += (dt / subSetp) * (lastSimVel + newVel) / 2
                        lastSimVel = newVel
                else:
                    lastSimPos += dt * (lastSimVel + newVel) / 2
                    lastSimVel = newVel
            else:
                newVel = realVel[i+1]
                lastSimPos += dt * (lastSimVel + newVel) / 2
                lastSimVel = newVel

            simVel[i + 1] = lastSimVel
            simPos[i + 1] = lastSimPos


        realVel = calcVelFromPos(realPos, realTime, 1)[1:-3]
        simVel = simVel[1:-3]
        linearPwmData = linearPwmData[1:]

        simError = realVel - simVel
        temp = [e**1 * sign(pwm) for e, pwm in zip(simError, linearPwmData)]
        temp = [calcSpikeResistantAvarage(temp[i:i + pwmChangeDelay], excluded=0.2)
                for i in range(0, len(temp) - pwmChangeDelay)]
        l = max(1, int(len(temp) * 0.05))
        temp = sorted(temp)[l:-l]
        errorSum = sum(temp) / len(temp)
        errorSum = (abs(errorSum)) * sign(errorSum)

        return (simVel, realVel, simError, errorSum)

    def showAdditionalDiagnosticPlots(self, color='g', skipCallToShow=False):
        pwmCompFun = SystemIdentificationObject.getPwmLinearizer(self.servoModelParameters[4])
        linearPwmList = [pwmCompFun.getY(x) for x in self.pwmList]

        fig = plt.figure(1)
        fig.suptitle('Inverse time constant over pwm')
        x = np.arange(linearPwmList[0], linearPwmList[-1], 4)
        y = renderPolynome(x, [self.servoModelParameters[3], self.servoModelParameters[0]])

        line = plt.plot(x, y, '--', color=color, alpha=0.4)
        if color is None:
            color=line[0].get_color()

        plt.plot(linearPwmList, self.aList, '+', color=color)

        fig = plt.figure(2)
        fig.suptitle('Pwm nonlinearity')
        if len(fig.get_axes()) == 0:
            plt.plot([0, 1023], [0, 0], 'k:', alpha=0.4)
            plt.plot([0, 1023], [0, 1023], 'k:', alpha=0.4)

        linearCompList = [i * 1023 / 255 for i in range(0, 256)]
        pwmFromLinComp = [max(0, pwmCompFun.getX(v, takeLowest=True)) for v in linearCompList]
        pwmFromLinComp[0] = 0
        plt.plot(pwmFromLinComp, linearCompList, '--', color=color)
        plt.plot(self.pwmList, self.pwmCompList, '+', color=color)
        plt.xlim(-20, 1043)
        plt.ylim(-20, 1043)

        fig = plt.figure(9)
        plt.plot(self.pwmCompDiagData[0][0], self.pwmCompDiagData[0][1], '+', color='r')
        plt.plot(self.pwmCompDiagData[1][0], self.pwmCompDiagData[1][1], '+', color='g')
        plt.plot(self.pwmCompDiagData[2][0], self.pwmCompDiagData[2][1], '+', color='b')

        fig = plt.figure(7)
        plt.plot(self.aDiscList, '+', color=color)

        fig = plt.figure(8)
        plt.plot(self.bDiscList, '+', color=color)

        fig = plt.figure(4)
        fig.suptitle('Cycle time error [s]')
        t = np.arange(len(self.dtError)) * self.dt
        plt.plot(t, self.dtError, color=color)

        fig = plt.figure(5)
        fig.suptitle('Model error in percent of max velocity')
        t = np.arange(len(self.simRefVel)) * self.dt
        temp = sorted([abs(v) for v in self.simVel])
        maxVel = temp[-len(temp)//100]
        simError = [100 * v / maxVel for v in self.simError]
        if len(fig.get_axes()) == 0:
            plt.plot(t, [0 for _ in simError], 'k--', alpha=0.5)
        plt.plot(t, simError, '+-', color=color, alpha=0.5)
        plt.ylim(-20, 20)

        if not skipCallToShow:
            plt.show()

    def plotServoSystemModel(self, box=None):
        fig = Figure(figsize=(5, 4), dpi=100)
        if box is None:
            fig = plt.figure(6)

        ax = fig.add_subplot()

        t = np.arange(len(self.simVel)) * self.dt
        simError = self.simError
        ax.plot(t, self.simRefVel, '--', color=(0.0, 0.8, 0.0, 0.5))
        ax.plot(t, self.simVel, '-', color=(0.0, 0.0, 0.0, 1.0))
        ax.plot(t, self.simRefVel, '+', color=(0.0, 0.4, 0.0, 0.5))
        ax.plot(t, [0 for _ in simError], '--', color=(0.0, 0.0, 0.0, 1.0))
        ax.plot(t, simError, '--', color=(0.4, 0.0, 0.0, 0.5))
        ax.plot(t, simError, '+', color=(0.8, 0.0, 0.0, 0.5))

        canvas = FigureCanvas(fig)
        canvas.set_size_request(600, 400)
        zoomWindow = self.dt * self.pwmChangeDelay * 8
        defaultXLims = [-t[-1] * 0.02, t[-1] * 1.02]
        xLims = defaultXLims
        ax.set_xlim(xLims[0], xLims[1])
        temp = sorted([abs(v) for v in self.simVel])
        maxVel = temp[-len(temp)//100]
        ax.set_ylim(-maxVel * 1.15, maxVel * 1.15)

        if box is None:
            return

        def onRelease(event):
            nonlocal xLims

            xLims = defaultXLims
            ax.set_xlim(xLims[0], xLims[1])
            canvas.draw()

        def onMove(event):
            nonlocal xLims

            if event.button != 1:
                onRelease(event)
                return

            x = event.xdata
            if not x is None:
                xt = (x - xLims[0]) / (xLims[1] - xLims[0])
                x = defaultXLims[0] + xt * (defaultXLims[1] - defaultXLims[0])
                xLims = [x - zoomWindow / 2, x + zoomWindow / 2]
                ax.set_xlim(xLims[0], xLims[1])
                canvas.draw()

        canvas.mpl_connect('button_press_event', onMove)
        canvas.mpl_connect('button_release_event', onRelease)
        canvas.mpl_connect('motion_notify_event', onMove)
        box.add(canvas)

        label = Gtk.Label(label=
            'Green curve shows the recorded velocity and the black shows the\n'
            'identified model\'s response. The error between them is shown in red.\n'
            'A good result is indicated by the error being small and\n'
            'looking like white noise.\n\n'
            'Click in the graph to zoom.')

        label.set_use_markup(True)
        label.set_margin_start(30)
        label.set_margin_end(50)
        label.set_margin_top(8)
        label.set_margin_bottom(10)
        label.set_xalign(0.0)
        box.add(label)

        box.show_all()

class KalmanFilter:
    # pylint: disable=too-few-public-methods
    def __init__(self, dt, aMat, bMat, cMat):
        # pylint: disable=too-many-locals, too-many-statements
        aMatEx = np.hstack((aMat, bMat))
        aMatEx = np.vstack((aMatEx, np.array([[0, 0, 1]])))

        bMatEx = np.vstack((bMat, np.array([[0]])))
        cMatEx = np.hstack((cMat, np.array([[0]])))

        self.aMat = aMatEx
        self.aMatInv = np.linalg.inv(aMatEx)
        self.bMat = bMatEx
        self.cMat = cMatEx

        aMatT = np.transpose(aMatEx)
        cMatT = np.transpose(cMatEx)

        x = range(1 * 4 * 20, 100 * 4 * 20, 4 * 20)
        vec1 = []
        vec2 = []
        vec3 = []
        for v in x:
            kalmanFilterSpeed = v
            poles = np.exp(dt * np.array([-1.0, -0.98, -0.96]) * kalmanFilterSpeed)
            plaseResult = scipy.signal.place_poles(aMatT,  cMatT, poles)

            vec1.append(plaseResult.gain_matrix[0, 0])
            vec2.append(plaseResult.gain_matrix[0, 1])
            vec3.append(plaseResult.gain_matrix[0, 2])

        p1 = np.polyfit(x, vec1, 6)
        p2 = np.polyfit(x, vec2, 6)
        p3 = np.polyfit(x, vec3, 6)


        self.polyK = np.vstack((p1, p2, p3))

def getModelDtFromConfigFileString(configFileAsString, configClassName):
    configClassString = getConfigClassString(configFileAsString, configClassName)

    dtPattern = re.compile(
            r'Eigen::Matrix3f\s+A;[\n\s]+A\s*<<\s*([^,;]*,){1}(?P<a01>[^,;]*)([^,;]*,){3}(?P<a11>[^,;]*)')
    temp = dtPattern.search(configClassString)
    if temp is not None:
        a01Str = temp.group('a01')
        a11Str = temp.group('a11')
        a01Str = re.sub(r'f', '', a01Str)
        a11Str = re.sub(r'f', '', a11Str)

        a01 = float(a01Str)
        a11 = float(a11Str)

        contineusA = (1.0 - a11) / a01

        if contineusA <= 0.01:
            return a01

        dt = -math.log(a11) / contineusA
        return dt

    raise Exception('Could not find model dt')

class ServoModel:
    """docstring for ServoModel"""
    def __init__(self, dt, systemModel):

        self.aMat, self.bMat = systemModel.getServoSystemMatrices(dt)
        self.cMat = np.array([[1.0, 0.0]])

        self.kalmanFilter = KalmanFilter(dt, self.aMat, self.bMat, self.cMat)

        _, _, self.friction, (_, linearPwmList) = systemModel.getServoSystemModelParameters(dt)

        self.pwmNonlinearityComp = PwmNonlinearityConfigHandler(pwmCompLookUp=linearPwmList)

        self.pwmToStallCurrent, self.backEmfCurrent = systemModel.getCurrentModelParameters()

    def  getControlParametersClassContentStr(self, indent):
        out = ''
        out += indent + '  public:\n'
        out += indent + '    //kalman filter observer vector\n'
        out += indent + '    static Eigen::Matrix<float, 3, 7> getKVector()\n'
        out += indent + '    {\n'
        out += indent + '        Eigen::Matrix<float, 3, 7> K;\n'
        out += indent + '        K << ' + printAsEigenInit(self.kalmanFilter.polyK, indent + '            ')
        out += '\n'
        out += indent + '        return K;\n'
        out += indent + '    }\n'
        out += '\n'
        out += indent + '    //system model A matrix\n'
        out += indent + '    static Eigen::Matrix3f getAMatrix()\n'
        out += indent + '    {\n'
        out += indent + '        Eigen::Matrix3f A;\n'
        out += indent + '        A << ' + printAsEigenInit(self.kalmanFilter.aMat, indent + '            ')
        out += '\n'
        out += indent + '        return A;\n'
        out += indent + '    }\n'
        out += '\n'
        out += indent + '    //system model invers A matrix\n'
        out += indent + '    static Eigen::Matrix3f getAInvMatrix()\n'
        out += indent + '    {\n'
        out += indent + '        Eigen::Matrix3f AInv;\n'
        out += indent + '        AInv << ' + printAsEigenInit(self.kalmanFilter.aMatInv, indent + '            ')
        out += '\n'
        out += indent + '        return AInv;\n'
        out += indent + '    }\n'
        out += '\n'
        out += indent + '    //system model B matrix\n'
        out += indent + '    static Eigen::Vector3f getBVector()\n'
        out += indent + '    {\n'
        out += indent + '        Eigen::Vector3f B;\n'
        out += indent + '        B << ' + printAsEigenInit(self.kalmanFilter.bMat, indent + '            ')
        out += '\n'
        out += indent + '        return B;\n'
        out += indent + '    }\n'
        out += '\n'
        out += indent + '    static bool internalFeedForwardEnabled()\n'
        out += indent + '    {\n'
        out += indent + '        return true;\n'
        out += indent + '    }\n'
        out += '\n'
        out += indent + '    //system model friction comp value\n'
        out += indent + '    static float getFrictionComp()\n'
        out += indent + '    {\n'
        out += indent + '        return ' + str(self.friction) + 'f;\n'
        out += indent + '    }\n'
        out += indent + '};'
        return out

    def writeModelToConfigFileString(self, configFileAsString, configClassName):
        configFileAsString = self.pwmNonlinearityComp.writeLinearizationFunctionToConfigFileString(
                configFileAsString, configClassName)

        configClassString = getConfigClassString(configFileAsString, configClassName)

        pwmToStallCurrentPattern = re.compile(r'((\s*)constexpr\s+float\s+pwmToStallCurrent\s*)\{[^\}]*\};')
        backEmfCurrentPattern = re.compile(r'((\s*)constexpr\s+float\s+backEmfCurrent\s*)\{[^\}]*\};')
        controlParametersPattern = re.compile(r'((\s*)class\s+ControlParameters\s+:.*\n\2\{)(.*\n)*?\2\};')

        temp = True
        temp = pwmToStallCurrentPattern.search(configClassString) is not None and temp
        temp = backEmfCurrentPattern.search(configClassString) is not None and temp
        temp = controlParametersPattern.search(configClassString) is not None and temp
        if temp:
            out = r'\1{' + str(self.pwmToStallCurrent) + r'f};'
            configClassString = re.sub(pwmToStallCurrentPattern, out, configClassString)

            out = r'\1{' + str(self.backEmfCurrent) + r'f};'
            configClassString = re.sub(backEmfCurrentPattern, out, configClassString)

            out = r'\1\n'
            out += self.getControlParametersClassContentStr(r'\2')
            configClassString = re.sub(controlParametersPattern, out, configClassString)

            configFileAsString = setConfigClassString(configFileAsString, configClassName, configClassString)

            return configFileAsString

        return ''

    def getGeneratedModel(self):
        out = ''
        out += '--------------------------------------------------------------------------------\n'
        out += '\n'
        out += '        constexpr float pwmToStallCurrent{' + str(self.pwmToStallCurrent) + 'f};\n'
        out += '        constexpr float backEmfCurrent{' + str(self.backEmfCurrent) + 'f};\n'
        out += '\n'
        out += '--------------------------------------------------------------------------------\n'
        out += '\n'
        out += self.pwmNonlinearityComp.getLinearizationFunction('        ') + '\n'
        out += '\n'
        out += '--------------------------------------------------------------------------------\n'
        out += '\n'
        out += '    class ControlParameters : public SetupConfigHolder::DefaultControlParameters\n'
        out += '    {\n'
        out += self.getControlParametersClassContentStr('    ') + '\n'
        out += '\n'
        out += '--------------------------------------------------------------------------------\n'

        return out

def createGuiBox(parent, nodeNr, getPortFun, configFilePath, configClassName):
    # pylint: disable=too-many-locals, too-many-statements
    with open(configFilePath, 'r', encoding='utf-8') as configFile:
        configFileAsString = configFile.read()

        try:
            SystemIdentificationObject.checkForPreviousCalibration(configFileAsString, configClassName)
        except Exception:
            dialog = Gtk.MessageDialog(
                    transient_for=parent,
                    flags=0,
                    message_type=Gtk.MessageType.ERROR,
                    buttons=Gtk.ButtonsType.YES_NO,
                    text='Pwm calibration not compatible with this configuration! Continue any way?',
            )
            dialog.format_secondary_text(
                ''
            )
            response = dialog.run()
            dialog.destroy()

            if response == Gtk.ResponseType.NO:
                return None

    calibrationBox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
    calibrationBox.set_margin_start(40)
    calibrationBox.set_margin_bottom(100)

    limitMovementButton = GuiFunctions.createToggleButton('Lock', getLowLev=True)
    limitMovementButton = (GuiFunctions.addTopLabelTo('<b>Limit movement</b>\n '
                'Only move around locked position to avoid end limits', limitMovementButton[0]),
            limitMovementButton[1])
    calibrationBox.pack_start(limitMovementButton[0], False, False, 0)

    startPos = None

    def onLockPosition(widget):
        nonlocal startPos
        if widget.get_active():
            try:
                with createServoManager(nodeNr, getPortFun()) as servoManager:
                    startPos = servoManager.servoArray[0].getPosition(True)
            except Exception as e:
                GuiFunctions.exceptionMessage(parent, e)
                widget.set_active(False)

        else:
            startPos = None

    limitMovementButton[1].connect('toggled', onLockPosition)

    outputModelDt = 0.0012
    motorSettleTime = 0.3
    minPwmValue = 0
    maxPwmValue = 1023

    try:
        with open(configFilePath, "r", encoding='utf-8') as configFile:
            configFileAsString = configFile.read()
            outputModelDt = getModelDtFromConfigFileString(configFileAsString, configClassName)
            outputModelDt = 0.0002 * int(round(outputModelDt / 0.0002))
    except Exception:
        pass


    motorSettleTimeScale = GuiFunctions.creatHScale(motorSettleTime, 0.1, 5.0, 0.1, getLowLev=True)
    motorSettleTimeScale = (GuiFunctions.addTopLabelTo('<b>Motor settle time</b>', motorSettleTimeScale[0]),
            motorSettleTimeScale[1])
    calibrationBox.pack_start(motorSettleTimeScale[0], False, False, 0)

    minPwmScale = GuiFunctions.creatHScale(minPwmValue, 0, 1023, 1, getLowLev=True)
    minPwmScale = GuiFunctions.addTopLabelTo('<b>Min motor pwm value</b>', minPwmScale[0]), minPwmScale[1]
    calibrationBox.pack_start(minPwmScale[0], False, False, 0)

    maxPwmScale = GuiFunctions.creatHScale(maxPwmValue, 0, 1023, 1, getLowLev=True)
    maxPwmScale = GuiFunctions.addTopLabelTo('<b>Max motor pwm value</b>', maxPwmScale[0]), maxPwmScale[1]
    calibrationBox.pack_start(maxPwmScale[0], False, False, 0)

    testButton = GuiFunctions.createButton('Test pwm value', getLowLev=True)

    dtSpinButton = GuiFunctions.creatSpinButton(outputModelDt * 1000, 0.6, 2.4, 0.2, getLowLev=True)
    dtSpinButton[1].set_margin_end(6)
    label = GuiFunctions.createLabel('ms')
    label.set_margin_start(6)
    dtSpinButton[0].pack_start(label, False, False, 0)
    dtSpinButton = GuiFunctions.addTopLabelTo('<b>System model cycle time</b>', dtSpinButton[0]), dtSpinButton[1]


    startButton = GuiFunctions.createButton('Start calibration', getLowLev=True)

    recordingProgressBar = GuiFunctions.creatProgressBar(label='Recording', getLowLev=True)
    analyzingProgressBar = GuiFunctions.creatProgressBar(label='Analyzing', getLowLev=True)

    testPwmValue = minPwmValue

    threadMutex = threading.Lock()

    def motorSettleTimeChanged(widget):
        nonlocal motorSettleTime

        with threadMutex:
            motorSettleTime = widget.get_value()

    def minPwmValueChanged(widget):
        nonlocal minPwmValue
        nonlocal testPwmValue

        with threadMutex:
            minPwmValue = widget.get_value()
            testPwmValue = minPwmValue

        if minPwmValue > maxPwmScale[1].get_value():
            maxPwmScale[1].set_value(minPwmValue)

    def maxPwmValueChanged(widget):
        nonlocal maxPwmValue
        nonlocal testPwmValue

        with threadMutex:
            maxPwmValue = widget.get_value()
            testPwmValue = maxPwmValue

        if maxPwmValue < minPwmScale[1].get_value():
            minPwmScale[1].set_value(maxPwmValue)

    motorSettleTimeScale[1].connect('value-changed', motorSettleTimeChanged)
    minPwmScale[1].connect('value-changed', minPwmValueChanged)
    maxPwmScale[1].connect('value-changed', maxPwmValueChanged)

    def resetGuiAfterCalibration():
        testButton[1].set_label('Test pwm value')
        testButton[1].set_sensitive(True)
        dtSpinButton[1].set_sensitive(True)
        startButton[1].set_label('Start calibration')
        startButton[1].set_sensitive(True)
        calibrationBox.remove(recordingProgressBar[0])
        calibrationBox.remove(analyzingProgressBar[0])
        motorSettleTimeScale[1].set_sensitive(True)
        minPwmScale[1].set_sensitive(True)
        maxPwmScale[1].set_sensitive(True)
        limitMovementButton[1].set_sensitive(True)

    runThread = False
    def testPwmRun(nodeNr, port):
        # pylint: disable=too-many-statements
        nonlocal runThread

        try:
            with createServoManager(nodeNr, port) as servoManager:
                t = 0.0
                doneRunning = False
                pwmDir = 1
                moveDir = 0

                def sendCommandHandlerFunction(dt, servoManager):
                    nonlocal pwmDir
                    nonlocal moveDir

                    servo = servoManager.servoArray[0]

                    pos = servo.getPosition(True)

                    if startPos is not None:
                        if pos - startPos < -1 and moveDir != -1:
                            moveDir = -1
                            pwmDir *= -1
                        elif pos - startPos > 1 and moveDir != 1:
                            moveDir = 1
                            pwmDir *= -1

                    servo.setOpenLoopControlSignal(testPwmValue * pwmDir, True)

                out = []

                def readResultHandlerFunction(dt, servoManager):
                    nonlocal t
                    nonlocal doneRunning

                    t += dt
                    servo = servoManager.servoArray[0]
                    out.append([t, servo.getVelocity()])

                    stop = False
                    with threadMutex:
                        if runThread is False:
                            stop = True

                    if stop or parent.isClosed:
                        servoManager.removeHandlerFunctions()
                        doneRunning = True

                servoManager.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction)

                while not doneRunning:
                    if not servoManager.isAlive():
                        break
                    time.sleep(0.1)

                servoManager.shutdown()

                data = np.array(out)

                def plotData(data):
                    dialog = Gtk.MessageDialog(
                            transient_for=parent,
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
                        fig = plt.figure(1)
                        fig.suptitle('Velocity')
                        plt.plot(data[:, 0], data[:, 1], 'r')
                        plt.show()

                GLib.idle_add(plotData, data)

        except Exception as e:
            GuiFunctions.exceptionMessage(parent, e)
        finally:
            GLib.idle_add(resetGuiAfterCalibration)

    testThread = None

    def onTestPwm(widget):
        nonlocal testThread
        nonlocal runThread

        if widget.get_label() == 'Test pwm value':
            dtSpinButton[1].set_sensitive(False)
            startButton[1].set_sensitive(False)
            limitMovementButton[1].set_sensitive(False)
            widget.set_label('Stop pwm test')
            with threadMutex:
                runThread = True
            testThread = threading.Thread(target=testPwmRun, args=(nodeNr, getPortFun(),))
            testThread.start()
        else:
            with threadMutex:
                runThread = False
            testThread.join()

    def onDtSpinButtonChange(widget):
        nonlocal outputModelDt
        outputModelDt = widget.get_value() * 0.001

    def updateRecordingProgressBar(fraction):
        recordingProgressBar[1].set_fraction(fraction)

    def updateAnalyzingProgressBar(fraction):
        analyzingProgressBar[1].set_fraction(fraction)

    def handleResults(systemIdentifier):
        dialog = Gtk.MessageDialog(
                transient_for=parent,
                flags=0,
                message_type=Gtk.MessageType.INFO,
                buttons=Gtk.ButtonsType.YES_NO,
                text='System identification done!',
        )
        dialog.format_secondary_text(
            "Do you want to plot extended data?"
        )
        response = dialog.run()
        dialog.destroy()

        if response == Gtk.ResponseType.YES:
            systemIdentifier.showAdditionalDiagnosticPlots()

        dialog = Gtk.MessageDialog(
                transient_for=parent,
                flags=0,
                message_type=Gtk.MessageType.INFO,
                buttons=Gtk.ButtonsType.YES_NO,
                text='System identification done!',
        )
        dialog.format_secondary_text(
            "Should the configuration be updated with the new model?"
        )
        systemIdentifier.plotServoSystemModel(dialog.get_message_area())
        dialog.get_widget_for_response(Gtk.ResponseType.YES).grab_focus()
        response = dialog.run()
        dialog.destroy()

        if response == Gtk.ResponseType.YES:
            servoModel = ServoModel(outputModelDt, systemIdentifier)

            with open(configFilePath, "r", encoding='utf-8') as configFile:
                configFileAsString = configFile.read()

                configFileAsString = servoModel.writeModelToConfigFileString(configFileAsString, configClassName)

                if configFileAsString != '':
                    with open(configFilePath, "w", encoding='utf-8') as configFile:
                        configFile.write(configFileAsString)
                        GuiFunctions.transferToTargetMessage(parent)

                        return

            dialog = Gtk.MessageDialog(
                    transient_for=parent,
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
            funEntry.set_text(servoModel.getGeneratedModel())
            box.add(funEntry)
            box.show_all()
            response = dialog.run()
            dialog.destroy()

    def startCalibrationRun(nodeNr, port):
        # pylint: disable=too-many-locals, too-many-statements
        nonlocal runThread

        try:
            sampleTime = max(0.004, motorSettleTime * 0.2 / 15)
            with createServoManager(nodeNr, port, dt=sampleTime) as servoManager:
                pwmSampleValues = []
                nr = 30
                for i in range(0, nr):
                    t = i / nr
                    t = t**0.5
                    pwmSampleValues.append(maxPwmValue - t * (maxPwmValue - minPwmValue))

                def sendCommandHandlerFunction(dt, servoManager):
                    return

                t = 0.0

                doneRunning = False
                i = 0
                lastI = -1
                pwm = None
                encDirDetectMem = [None, 0]
                outEncDir = 0
                lastDirChangePos = 0
                lastPwmSignFlipp = 0.0

                out = []

                if os.path.isfile('sysTestDataToLoad.txt') and t == 0.0:
                    dialog = Gtk.MessageDialog(
                            transient_for=parent,
                            flags=0,
                            message_type=Gtk.MessageType.INFO,
                            buttons=Gtk.ButtonsType.YES_NO,
                            text='Found file: "sysTestDataToLoad.txt"!\n'
                                'Should this file be loaded instead of\n'
                                'running a new calibration?',
                    )
                    response = dialog.run()
                    dialog.destroy()

                    if response == Gtk.ResponseType.YES:
                        out = np.loadtxt('sysTestDataToLoad.txt')
                        doneRunning = True

                def readResultHandlerFunction(dt, servoManager):
                    # pylint: disable=too-many-branches
                    nonlocal t
                    nonlocal doneRunning
                    nonlocal pwm
                    nonlocal i
                    nonlocal lastI
                    nonlocal startPos
                    nonlocal encDirDetectMem
                    nonlocal outEncDir
                    nonlocal lastDirChangePos
                    nonlocal lastPwmSignFlipp

                    stop = False

                    servo = servoManager.servoArray[0]
                    pos = servo.getPosition(True)

                    if encDirDetectMem[0] is None:
                        encDirDetectMem[0] = pos

                    diff = pos - encDirDetectMem[0]
                    diff2 = diff - encDirDetectMem[1]
                    velSign = sign(diff)
                    lastVelSign = sign(encDirDetectMem[1])
                    lastPos = encDirDetectMem[0]
                    encDirDetectMem[0] = pos
                    encDirDetectMem[1] = diff
                    outEncDir += diff2 * servo.getFeedforwardU()

                    pwmSignFlipp = 0.5
                    if startPos is not None:
                        deadZone = 0.0
                        maxDiffFromStart = 1.0
                        if lastVelSign != velSign:
                            lastDirChangePos = lastPos

                        diff = 2 * pos - lastDirChangePos - startPos
                        scaledPosDiff = diff / maxDiffFromStart
                        if outEncDir < 0:
                            scaledPosDiff *= -1.0


                        if abs(scaledPosDiff) <= deadZone:
                            scaledPosDiff = 0.0
                        elif abs(scaledPosDiff) < 1.0 and sign(scaledPosDiff) != velSign:
                            scaledPosDiff = 0.0
                        else:
                            scaledPosDiff = (abs(scaledPosDiff) - deadZone) / (1.0 - deadZone) * sign(scaledPosDiff)


                        pwmSignFlipp = min(max(0.5 * scaledPosDiff + 0.5, 0.0), 1.0)

                        if lastPwmSignFlipp != pwmSignFlipp and pwmSignFlipp in (0.0, 1.0):
                            lastI = i - 1

                        lastPwmSignFlipp = pwmSignFlipp

                    if pwm is not None:
                        temp = pos
                        if startPos is not None:
                            temp -= startPos
                        d = [t, servo.getPosition(False) / servo.getScaling(),
                                servo.getFeedforwardU(),
                                servo.getTime()]
                        out.append(d)

                    i = int(t / (motorSettleTime * 0.2))
                    if i // 50 < len(pwmSampleValues):

                        if lastI < i:
                            pwm = pwmSampleValues[i // 50]
                            if random.random() < pwmSignFlipp:
                                pwm = -pwm
                        lastI = i
                    else:
                        pwm = 0
                        stop = True

                    servo.setOpenLoopControlSignal(pwm, True)

                    GLib.idle_add(updateRecordingProgressBar,
                            (t / (motorSettleTime * 0.2 * 50)) / len(pwmSampleValues))

                    with threadMutex:
                        if runThread is False:
                            stop = True

                    if stop or parent.isClosed:
                        servoManager.removeHandlerFunctions()
                        doneRunning = True
                        return

                    t += dt

                if not doneRunning:
                    servoManager.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction)

                while not doneRunning:
                    if not servoManager.isAlive():
                        runThread = False
                        break
                    time.sleep(0.1)

                servoManager.shutdown()

                def shouldAbort():
                    with threadMutex:
                        if runThread is False:
                            return True
                    return False

                lastFraction = 0.0
                def updateProgress(fraction):
                    nonlocal lastFraction

                    if abs(fraction - lastFraction) > 0.01:
                        lastFraction = fraction
                        GLib.idle_add(updateAnalyzingProgressBar, fraction)

                if runThread is True:
                    data = np.array(out)
                    np.savetxt('lastSysTestData.txt', data)
                    systemIdentifier = SystemIdentificationObject(data,
                                shouldAbort=shouldAbort,
                                updateProgress=updateProgress)
                    GLib.idle_add(handleResults, systemIdentifier)

        except Exception as e:
            GuiFunctions.exceptionMessage(parent, e)
        finally:
            GLib.idle_add(resetGuiAfterCalibration)

    def onStartCalibration(widget):
        nonlocal testThread
        nonlocal runThread

        if widget.get_label() == 'Start calibration':
            with open(configFilePath, 'r', encoding='utf-8') as configFile:
                configFileAsString = configFile.read()

                previousCalibrationDetected = False
                try:
                    previousCalibrationDetected = PwmNonlinearityConfigHandler.checkForPreviousCalibration(
                            configFileAsString, configClassName)
                except Exception:
                    pass

                if previousCalibrationDetected:
                    dialog = Gtk.MessageDialog(
                            transient_for=parent,
                            flags=0,
                            message_type=Gtk.MessageType.ERROR,
                            buttons=Gtk.ButtonsType.YES_NO,
                            text='Pwm calibration already done for this configuration!',
                    )
                    dialog.format_secondary_text(
                        'Pwm linarization only works on configurations without previous calibration.\n\n'
                        'Should the calibration be reset?'
                    )
                    response = dialog.run()
                    dialog.destroy()

                    if response == Gtk.ResponseType.NO:
                        return

                    configFileAsString = PwmNonlinearityConfigHandler.resetPreviousCalibration(
                            configFileAsString, configClassName)
                    with open(configFilePath, 'w', encoding='utf-8') as configFile:
                        configFile.write(configFileAsString)
                        GuiFunctions.transferToTargetMessage(parent)
                    return

            widget.set_label('Abort calibration')

            recordingProgressBar[1].set_fraction(0.0)
            analyzingProgressBar[1].set_fraction(0.0)
            testButton[1].set_sensitive(False)
            dtSpinButton[1].set_sensitive(False)
            calibrationBox.pack_start(recordingProgressBar[0], False, False, 0)
            calibrationBox.pack_start(analyzingProgressBar[0], False, False, 0)
            motorSettleTimeScale[1].set_sensitive(False)
            minPwmScale[1].set_sensitive(False)
            maxPwmScale[1].set_sensitive(False)
            limitMovementButton[1].set_sensitive(False)

            calibrationBox.show_all()

            with threadMutex:
                runThread = True
            testThread = threading.Thread(target=startCalibrationRun, args=(nodeNr, getPortFun(),))
            testThread.start()
        else:
            with threadMutex:
                runThread = False
            testThread.join()

    testButton[1].connect('clicked', onTestPwm)
    dtSpinButton[1].connect('changed', onDtSpinButtonChange)
    startButton[1].connect('clicked', onStartCalibration)
    calibrationBox.pack_start(testButton[0], False, False, 0)
    calibrationBox.pack_start(dtSpinButton[0], False, False, 0)
    calibrationBox.pack_start(startButton[0], False, False, 0)
    calibrationBox.show_all()

    return calibrationBox

def main():
    dataLists = []

    #dataLists.append([
    #    np.loadtxt('sysTestDataServoNr1T02.txt')])
    #dataLists.append([
    #    np.loadtxt('sysTestDataServoNr2T02.txt')])

    for data in dataLists:
        systemIdentifier = SystemIdentificationObject([], additionalData=data)
        systemIdentifier.showAdditionalDiagnosticPlots(color=None, skipCallToShow=True)

    plt.show()

if __name__ == '__main__':
    main()
