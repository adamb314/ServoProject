'''
Module for calibrating pwm nonlinearity
'''
# pylint: disable=duplicate-code, too-many-lines

import os
from ServoProjectModules.CalibrationAnalyzers.Helper import *  # pylint: disable=wildcard-import, unused-wildcard-import

def highPassFilter(v, a):
    out = 0 * v + v[0]
    for i in range(0, len(v) - 1):
        out[i + 1] = a * out[i] + a * (v[i + 1] - v[i])
    return out

class SystemIdentificationObject:
    # pylint: disable=too-many-instance-attributes
    def __init__(self, data, *, ad=None, bd=None, friction=None, pwmOffset=None, dt=None):
        # pylint: disable=too-many-locals
        if len(data) == 0:
            self.dt = dt
            self.velData = np.array([[0], [0]])
            self.pwmData = np.array([[0], [0]])

            self.servoModelParameters = np.array([ad, bd, friction, pwmOffset])
            return

        self.rawData = data[:]
        self.dt = data[1, 0] - data[0, 0]

        t = np.array(data[:, 0])
        wallTime = np.array(data[:, 3])
        posData = data[:, 1]
        pwmData = np.array(data[:, 2])

        dtError = wallTime - t
        dtError += np.arange(0, 1.0, 1.0 / len(dtError)) * np.mean(wallTime[-100:] - t[-100:])
        dtError -= np.mean(sorted(dtError)[len(dtError) // 5: -len(dtError) // 5])
        outOfDtSync = [abs(d / self.dt) >= 0.5 for d in dtError]
        self.dtError = dtError

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

        self.pwmChangeDelay = findPwmChangeDelay(pwmData)

        self.dStep = self.pwmChangeDelay // 3

        def calcVelFromPos(posData, dt, dStep):
            velData = np.array([0.0 for d in range(0, len(posData))])
            for i, d in enumerate(zip(posData[self.dStep:], posData[0:-self.dStep])):
                diffs = [d[0] - d[1] for d in zip(posData[i+1:i+1+self.dStep], posData[i:i+self.dStep])]
                diffs.sort()
                medianDiff = diffs[len(diffs) // 2]
                if len(diffs) % 2 == 0:
                    medianDiff += diffs[len(diffs) // 2 + 1]
                    medianDiff = medianDiff / 2
                velData[i + self.dStep] = medianDiff / self.dt

            return velData

        for i, d in enumerate(outOfDtSync):
            if d:
                pwmData[i - 1] = math.nan
                pwmData[i] = math.nan
                pwmData[i + 1] = math.nan

        velData = calcVelFromPos(posData, self.dt, self.dStep)
        velData = velData[self.dStep:]
        pwmData = pwmData[self.dStep:]

        self.velData = velData
        self.pwmData = pwmData

        self.servoModelParameters = self.identifyServoSystemModel()
        self.simVel, self.simRefVel, self.simT = self.getSimResults()

    @staticmethod
    def checkForPreviousCalibration(configFileAsString, configClassName):
        return PwmNonlinearityConfigHandler.checkForPreviousCalibration(configFileAsString, configClassName)

    def getServoSystemMatrices(self, outputDt):
        dt = self.dt * self.dStep
        discreteA = self.servoModelParameters[0]
        discreteB = self.servoModelParameters[1]

        #discreteA = math.exp(-contineusA * dt) =>
        contineusA = -math.log(discreteA) / dt

        #discreteB = contineusB * (1.0 - discreteA) / contineusA =>
        contineusB = discreteB * contineusA / (1.0 - discreteA)

        scaledDiscreteA = math.exp(outputDt * -contineusA)
        scaledDiscreteB = contineusB * (1.0 - scaledDiscreteA) / contineusA

        scaledDiscretePosA = (1.0 - scaledDiscreteA) / contineusA
        scaledDiscretePosB = contineusB * (outputDt - scaledDiscretePosA) / contineusA

        aMat = np.array([[1.0, scaledDiscretePosA], [0.0, scaledDiscreteA]])
        bMat = np.array([[scaledDiscretePosB], [scaledDiscreteB]])

        return (aMat, bMat)

    def getServoSystemModelParameters(self, outputDt):
        aMat, bMat = self.getServoSystemMatrices(outputDt)

        return np.array([aMat[1, 1], bMat[1, 0], self.servoModelParameters[2], self.servoModelParameters[3]])

    def getCurrentModelParameters(self):
        currentModelParams = np.array([1.0, 0.0])
        return currentModelParams

    def identifyServoSystemModel(self):
        # pylint: disable=too-many-locals, too-many-statements
        index = -1
        pwmVec = []
        phi2SumList = []
        yPhiSumList = []
        phiSumList = []
        ySumList = []
        nrOfSumList = []

        velData = self.velData
        pwmData = self.pwmData

        pltVec = []

        lastPwm = pwmData[self.dStep]
        if not math.isnan(lastPwm):
            index += 1
            pwmVec.append(abs(lastPwm))
        for i, d in enumerate(zip(velData[self.dStep:], velData[0:-self.dStep], pwmData[self.dStep-1:-1])):
            pwmSlice = pwmData[max(i - self.dStep, 0):i + self.dStep]
            constPwm = all(pwmSlice[0] == d for d in pwmSlice[1:])
            if constPwm and d[0] * d[1] > 0:
                if abs(lastPwm) != abs(d[2]):
                    index += 1
                    pwmVec.append(abs(d[2]))
                lastPwm = d[2]

                pltVec.append(d[0])

                phi = np.matrix([[d[1]], [d[2]], [sign(d[1])], [sign(d[2])]])
                y = d[0]

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

        phi2SumListCombined = []
        yPhiSumListCombined = []
        phiSumListCombined = []
        ySumListCombined = []
        nrOfSumListCombined = []
        pwmVecCombined = []

        for nr in [len(phi2SumList), len(phi2SumList) // 2]:
            phi2SumListCombined += sumAdjacent(phi2SumList, nr)
            yPhiSumListCombined += sumAdjacent(yPhiSumList, nr)
            phiSumListCombined += sumAdjacent(phiSumList, nr)
            ySumListCombined += sumAdjacent(ySumList, nr)
            nrOfSumListCombined += sumAdjacent(nrOfSumList, nr)
            pwmVecCombined += avarageAdjacent(pwmVec, nr)

        phi2SumList = phi2SumListCombined
        yPhiSumList = yPhiSumListCombined
        phiSumList = phiSumListCombined
        ySumList = ySumListCombined
        nrOfSumList = nrOfSumListCombined
        pwmVec = pwmVecCombined

        phiCovList = []
        yPhiCovList = []
        for phi2Sum, yPhiSum, phiSum, ySum, nr in zip(phi2SumList, yPhiSumList, phiSumList, ySumList, nrOfSumList):
            phiCovList.append(phi2Sum * nr - phiSum * np.transpose(phiSum))
            yPhiCovList.append(yPhiSum * nr - phiSum * np.transpose(ySum))

        aList = []
        bList = []
        fList = []
        pList = []

        for d in zip(phiCovList, yPhiCovList):
            params = np.linalg.solve(d[0], d[1])
            pwmOffset = -params[3, 0] / params[1, 0]
            ad = params[0, 0]
            bd = params[1, 0] * (1023 - pwmOffset) / 1023
            friction = -params[2, 0] / bd

            aList.append(ad)
            bList.append(bd)
            fList.append(friction)
            pList.append(pwmOffset)

        self.pwmVec = pwmVec
        self.aList = aList
        self.bList = bList
        self.fList = fList
        self.pList = pList

        return np.array([aList[0], bList[0], fList[0], pList[0]])

    def getSimResults(self):
        # pylint: disable=too-many-locals
        ad, bd, friction, pwmOffset = self.getServoSystemModelParameters(self.dt)

        data = self.rawData
        realPos = data[2:, 1]
        pwmData = data[2:, 2]

        simPos = 0 * realPos + realPos[0]
        lastSimVel = 0
        lastSimPos = simPos[0]
        for i, pwm in enumerate(pwmData[0:-1]):
            pwmScale = (1023 - pwmOffset) / 1023
            #pwm = linearPwm * pwmScale + pwmOffset
            linearPwm = (abs(pwm) - pwmOffset) / pwmScale * sign(pwm)

            newVel = (ad * lastSimVel +
                bd * (linearPwm - friction * sign(lastSimVel)))
            lastSimPos += self.dt * (lastSimVel + newVel) / 2
            lastSimVel = newVel

            simPos[i + 1] = lastSimPos

        realVel = 0 * realPos
        simVel = 0 * realPos
        for i, d in enumerate(zip(realPos[1:], realPos[0:-1], simPos[1:], simPos[0:-1])):
            realVel[i + 1] = (d[0] - d[1]) / self.dt
            simVel[i + 1] = (d[2] - d[3]) / self.dt

        highPassConst = self.pwmChangeDelay * 5 / (self.pwmChangeDelay * 5 + 1)
        realVel = highPassFilter(realVel[1:], highPassConst)
        simVel = highPassFilter(simVel[1:], highPassConst)

        t = np.arange(len(simVel)) * self.dt

        return (simVel, realVel, t)

    def showAdditionalDiagnosticPlots(self, color=None):
        plotColor=color
        if not plotColor:
            plotColor='g'

        plt.figure(1)
        plt.plot(self.simT, self.simRefVel, '--', color=plotColor)
        plt.plot(self.simT, self.simVel, 'k')

        plt.figure(2)
        plt.plot(self.pwmVec[1:], self.aList[1:], '+', color=plotColor)
        plt.plot([self.pwmVec[1], self.pwmVec[-1]], [self.aList[0], self.aList[0]], color=plotColor)
        plt.figure(3)
        plt.plot(self.pwmVec[1:], self.bList[1:], '+', color=plotColor)
        plt.plot([self.pwmVec[1], self.pwmVec[-1]], [self.bList[0], self.bList[0]], color=plotColor)
        plt.figure(4)
        plt.plot(self.pwmVec[1:], self.fList[1:], '+', color=plotColor)
        plt.plot([self.pwmVec[1], self.pwmVec[-1]], [self.fList[0], self.fList[0]], color=plotColor)
        plt.figure(5)
        plt.plot(self.pwmVec[1:], self.pList[1:], '+', color=plotColor)
        plt.plot([self.pwmVec[1], self.pwmVec[-1]], [self.pList[0], self.pList[0]], color=plotColor)

        plt.figure(6)
        pwmChangeDelay = self.pwmChangeDelay
        simRefVelFiltered = np.array([np.mean(self.simRefVel[i:i+pwmChangeDelay]) for i in range(0, len(self.simRefVel)
                - pwmChangeDelay)])
        simVelFiltered = np.array([np.mean(self.simVel[i:i+pwmChangeDelay]) for i in range(0, len(self.simVel)
                - pwmChangeDelay)])
        t = np.arange(len(simVelFiltered)) * self.dt
        temp = sorted(simRefVelFiltered)
        velAmplitude = max(np.abs([temp[len(temp) // 100], temp[-len(temp) // 100]]))
        plt.plot(t, (np.abs(simRefVelFiltered) - np.abs(simVelFiltered)) / velAmplitude, color=plotColor)

        plt.figure(7)
        plt.plot(self.velData, color=plotColor)

        plt.figure(8)
        plt.plot(self.dtError, color=plotColor)

        if not color:
            plt.show()

    def plotServoSystemModel(self, box):
        fig = Figure(figsize=(5, 4), dpi=100)
        ax = fig.add_subplot()

        ax.plot(self.simT, self.simRefVel, 'g--')
        ax.plot(self.simT, self.simVel, 'k')
        ax.set_xlim(0, self.dt * self.pwmChangeDelay * 100)

        canvas = FigureCanvas(fig)
        canvas.set_size_request(600, 400)
        box.add(canvas)

        label = Gtk.Label(label='Green curve is real system and black is model. A good result\n'
                            'is indicated by the black curve resembling the green.')
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

        dt = -math.log(a11) / contineusA
        return dt

    raise Exception('Could not find model dt')

class ServoModel:
    """docstring for ServoModel"""
    def __init__(self, dt, systemModel):

        self.aMat, self.bMat = systemModel.getServoSystemMatrices(dt)
        self.cMat = np.array([[1.0, 0.0]])

        self.kalmanFilter = KalmanFilter(dt, self.aMat, self.bMat, self.cMat)

        _, _, self.friction, pwmOffset = systemModel.getServoSystemModelParameters(dt)

        self.pwmNonlinearityComp = PwmNonlinearityConfigHandler(pwmOffset=pwmOffset)

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

        controlParametersPattern = re.compile(r'((\s*)class\s+ControlParameters\s+:.*\n\2\{)(.*\n)*?\2\};')

        temp = controlParametersPattern.search(configClassString) is not None
        if temp:
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

    def handleResults(data):
        systemIdentifier = SystemIdentificationObject(data)

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
            with createServoManager(nodeNr, port, dt=0.004) as servoManager:
                pwmSampleValues = []
                nr = 20
                for i in range(0, nr):
                    pwmSampleValues.append(i * (minPwmValue - maxPwmValue) / nr + maxPwmValue)

                def sendCommandHandlerFunction(dt, servoManager):
                    return

                t = 0.0
                wallTime = 0.0
                timestamp = time.monotonic()

                doneRunning = False
                i = 0
                lastI = -1
                pwm = None
                encDirDetectMem = [None, 0]
                outEncDir = 0
                lastDirChangePos = 0
                lastPwmSignFlipp = 0.0

                out = []

                if os.path.isfile('sysTestDataToLoad.txt'):
                    out = np.loadtxt('sysTestDataToLoad.txt')
                    doneRunning = True

                def readResultHandlerFunction(dt, servoManager):
                    # pylint: disable=too-many-branches
                    nonlocal t
                    nonlocal wallTime
                    nonlocal timestamp
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
                                wallTime]
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

                    newTimestamp = time.monotonic()
                    wallTime += newTimestamp - timestamp
                    timestamp = newTimestamp

                if not doneRunning:
                    servoManager.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction)

                while not doneRunning:
                    if not servoManager.isAlive():
                        runThread = False
                        break
                    time.sleep(0.1)

                servoManager.shutdown()

                if runThread is True:
                    data = np.array(out)
                    np.savetxt('lastSysTestData.txt', data)
                    GLib.idle_add(handleResults, data)

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
            testButton[1].set_sensitive(False)
            dtSpinButton[1].set_sensitive(False)
            calibrationBox.pack_start(recordingProgressBar[0], False, False, 0)
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
    data = np.loadtxt('lastSysTestDataDtCheck1.txt')
    systemIdentifier = SystemIdentificationObject(data)
    systemIdentifier.showAdditionalDiagnosticPlots(color='r')
    data = np.loadtxt('lastSysTestDataDtCheck2.txt')
    systemIdentifier = SystemIdentificationObject(data)
    systemIdentifier.showAdditionalDiagnosticPlots(color='g')
    data = np.loadtxt('lastSysTestDataDtCheck3.txt')
    systemIdentifier = SystemIdentificationObject(data)
    systemIdentifier.showAdditionalDiagnosticPlots(color='b')
    plt.show()

if __name__ == '__main__':
    main()
