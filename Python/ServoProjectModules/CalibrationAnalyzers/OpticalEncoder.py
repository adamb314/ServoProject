'''
Module for calibrating the optical encoder
'''
# pylint: disable=too-many-lines, duplicate-code

import os
from ServoProjectModules.CalibrationAnalyzers.Helper import *  # pylint: disable=wildcard-import, unused-wildcard-import

def wrapAroundSegment(data, i, window):
    l = len(data)
    segStart = int(round(i - window / 2))
    segEnd = int(round(i + window / 2 + 1))
    seg = list(data[min(l, segStart+l):])
    seg += list(data[max(0, segStart):min(l, segEnd)])
    seg += list(data[0:max(0, segEnd-l)])

    return seg

class OpticalEncoderDataVectorGenerator:
    # pylint: disable=too-many-instance-attributes
    _aVecPattern = re.compile(
            r'(?P<beg>.*createMainEncoderHandler\(\)\s*\{(?:.*\n)*?\s*(constexpr)?\s+'
            r'(static)?\s+std\s*::\s*array\s*<\s*uint16_t\s*,\s*)'
            r'\d+(?P<end>\s*>\s*aVec\s*=\s*)\{(?P<vec>[\d\s,]*)\};')
    _bVecPattern = re.compile(
            r'(?P<beg>.*createMainEncoderHandler\(\)\s*\{(?:.*\n)*?\s*(constexpr)?\s+'
            r'(static)?\s+std\s*::\s*array\s*<\s*uint16_t\s*,\s*)'
            r'\d+(?P<end>\s*>\s*bVec\s*=\s*)\{(?P<vec>[\d\s,]*)\};')

    def __init__(self, data, configFileAsString='', configClassName='', *,
            constVelIndex=10000, shouldAbort=None, updateProgress=None):
        # pylint: disable=too-many-locals, too-many-branches, too-many-statements

        self.data = data[:, 0:3]
        self.constVelIndex = constVelIndex

        self.shouldAbort = shouldAbort
        self.updateProgress = updateProgress
        if self.updateProgress is None:
            self.updateProgress = lambda _ : None

        def removeSensorTrends(vec, trendLength=2000):
            def getAvgMinMax(vec, p):
                temp = sorted(vec)
                l = max(1, int(len(temp) * p))
                return np.mean(temp[0:l]), np.mean(temp[-l:])
            xMin = []
            xMax = []
            avgMin = []
            avgMax = []
            segNr = len(vec) // trendLength
            for i in range(self.constVelIndex // trendLength + 1, segNr):
                l = len(vec) // segNr
                minVal, maxVal = getAvgMinMax(vec[i*l:(i+1)*l], 0.01)
                halfMinVal, halfMaxVal = getAvgMinMax(vec[i*l:int((i+0.5)*l)], 0.01)
                xMin.append((i+1)*l if halfMinVal > minVal else i*l)
                xMax.append((i+1)*l if halfMaxVal < maxVal else i*l)
                avgMin.append(minVal)
                avgMax.append(maxVal)

            minFun = PiecewiseLinearFunction(xMin, avgMin)
            maxFun = PiecewiseLinearFunction(xMax, avgMax)
            minRef = minFun.getY(0)
            maxRef = maxFun.getY(0)
            refDiff = maxRef - minRef

            def rescale(i, val):
                minVal = minFun.getY(i)
                maxVal = maxFun.getY(i)
                valDiff = maxVal - minVal
                return refDiff / valDiff * (val - maxVal) + maxRef

            return np.array([rescale(i, v) for i, v in enumerate(vec)])

        self.data[constVelIndex:, 1] = removeSensorTrends(self.data[constVelIndex:, 1], trendLength=2000)
        self.data[constVelIndex:, 2] = removeSensorTrends(self.data[constVelIndex:, 2], trendLength=2000)

        self.updateProgress(0.1)

        def detectInvertedRotation(data):
            a0 = data[0, 1]
            b0 = data[0, 2]
            armed = 0
            startOfFirstRotation = None
            endOfFirstRotation = None
            meanCost = sum(((d[0] - a0)**2 + (d[1] -b0)**2 for d in data[:, 1:3])) / len(data)

            for i, d in enumerate(data[:, 1:3]):
                c = (d[0] - a0)**2 + (d[1] -b0)**2

                if startOfFirstRotation is None:
                    if c > meanCost:
                        startOfFirstRotation = i
                elif endOfFirstRotation is None:
                    if armed == 0:
                        if c > meanCost * 1.5:
                            armed = 1
                    elif armed == 1:
                        if c < meanCost * 0.5:
                            armed = 2
                    else:
                        if c > meanCost:
                            endOfFirstRotation = i
                            break

            if startOfFirstRotation is None or endOfFirstRotation is None:
                raise Exception('No movement detected')

            startIndex = startOfFirstRotation

            firstRotationData = data[startIndex:endOfFirstRotation]

            temp = [d[1] for d in firstRotationData]
            minAIndex = temp.index(min(temp))
            temp = [d[2] for d in firstRotationData]
            maxBIndex = temp.index(max(temp))

            l = len(firstRotationData)
            inverted = ((maxBIndex - minAIndex + l/2) % l - l/2) < 0
            return inverted

        invertedDir = detectInvertedRotation(self.data[0:constVelIndex])

        self.updateProgress(0.2)

        def filterOutStationarySegments(data):
            def calcCov(a1, b1, a0, b0):
                return (a1 - a0)**2 + (b1 - b0)**2

            filteredData = []
            filterSeg = []
            for d in data:
                if len(filterSeg) < 10:
                    filterSeg.append(d)
                else:
                    s = filterSeg[0]
                    cov = sum((calcCov(d[1], d[2], s[1], s[2]) for d in filterSeg[1:]))

                    if cov > 10000:
                        filteredData += filterSeg

                    filterSeg = []
            return filteredData

        self.nonStationaryData = filterOutStationarySegments(self.data[constVelIndex:])

        self.updateProgress(0.3)

        self.fullLengthAVector, self.fullLengthBVector = self.genVec(self.nonStationaryData)

        self.updateProgress(0.7)

        if invertedDir:
            self.fullLengthAVector = self.fullLengthAVector[::-1]
            self.fullLengthBVector = self.fullLengthBVector[::-1]

        outputSize = 2048
        step = len(self.fullLengthAVector) / outputSize
        self.aVec = [np.mean(wrapAroundSegment(self.fullLengthAVector, i*step, step))
                for i in range(0, outputSize)]
        self.bVec = [np.mean(wrapAroundSegment(self.fullLengthBVector, i*step, step))
                for i in range(0, outputSize)]

        self.oldAVec = None
        self.oldBVec = None

        def getShift(aVec, bVec, refAVec, refBVec):
            aWeights = [1] * len(aVec)
            bWeights = [1] * len(aVec)

            calibrationData = (aVec, bVec, aWeights, bWeights)

            posDiffs = [
                (i - OpticalEncoderDataVectorGenerator.calculatePosition(
                    refAVec[i], refBVec[i], calibrationData)[0])%len(aVec)
                for i in range(0, len(aVec), 128)]

            shift = int(round(sum(posDiffs) / len(posDiffs)))
            return shift

        def shiftVec(vec, shift):
            temp = []
            for d in vec:
                temp.append(d)
            return temp[-shift:] + temp[0:-shift]

        configClassString = getConfigClassString(configFileAsString, configClassName)

        if configClassString != '':
            temp1 = OpticalEncoderDataVectorGenerator._aVecPattern.search(configClassString)
            temp2 = OpticalEncoderDataVectorGenerator._bVecPattern.search(configClassString)

            if temp1 is not None and temp2 is not None:
                oldAVecStr = temp1.group('vec')
                oldBVecStr = temp2.group('vec')
                oldAVecStr = '[' + oldAVecStr + ']'
                oldBVecStr = '[' + oldBVecStr + ']'

                self.oldAVec = eval(oldAVecStr)  # pylint: disable=eval-used
                self.oldBVec = eval(oldBVecStr)  # pylint: disable=eval-used

                if len(self.oldAVec) <= 1:
                    self.oldAVec = None
                if len(self.oldBVec) <= 1:
                    self.oldBVec = None

        if self.oldAVec is not None and self.oldBVec is not None:
            bestFittShift = getShift(self.aVec, self.bVec, self.oldAVec, self.oldBVec)
            self.aVecShifted = shiftVec(self.aVec, bestFittShift)
            self.bVecShifted = shiftVec(self.bVec, bestFittShift)
        else:
            self.aVecShifted = self.aVec
            self.bVecShifted = self.bVec

        self.updateProgress(1.0)

    def genVec(self, data):
        # pylint: disable=too-many-locals, too-many-statements
        dataSumDiff = [(d[0], d[1]-d[2], d[1]+d[2]) for d in data]
        sortedDataByA = sorted(dataSumDiff, key=lambda d: d[1])

        cutI = int(round(len(sortedDataByA)/4))
        dataMinMaxB = sortedDataByA[cutI:-cutI]
        meanB = sum((b for _, _, b in dataMinMaxB)) / len(dataMinMaxB)
        dataMinB = [d for d in dataMinMaxB if d[2] < meanB]
        dataMaxB = [d for d in dataMinMaxB if d[2] >= meanB]

        dataMinMaxA = sortedDataByA[0:cutI] + sortedDataByA[-cutI:]
        dataMinMaxA = sorted(dataMinMaxA, key=lambda d: d[2])
        meanA = sum((a for _, a, _ in dataMinMaxA)) / len(dataMinMaxA)
        dataMinA = [d for d in dataMinMaxA if d[1] < meanA]
        dataMaxA = [d for d in dataMinMaxA if d[1] >= meanA]

        subDataLengths = []
        dataSumDiff = dataMinA
        subDataLengths.append(len(dataSumDiff))
        dataSumDiff += dataMaxB
        subDataLengths.append(len(dataSumDiff))
        dataSumDiff += dataMaxA[::-1]
        subDataLengths.append(len(dataSumDiff))
        dataSumDiff += dataMinB[::-1]
        subDataLengths.append(len(dataSumDiff))

        data = [(d[0], (d[2]+d[1])/2, (d[2]-d[1])/2) for d in dataSumDiff]

        l = len(data)
        data = data[-l//8:] + data[0:-l//8]
        #data = data[l//8:] + data[0:l//8]
        #data = data[::-1]

        aVec = [a for _, a, _ in data]
        bVec = [b for _, _, b in data]

        return (np.array(aVec), np.array(bVec))

    @staticmethod
    def calcSensorWeights(aVec, bVec):
        vecSize = len(aVec)

        minAIndex = aVec.index(min(aVec))
        maxBIndex = bVec.index(max(bVec))

        dirSign = sign((maxBIndex - minAIndex + vecSize/2) % vecSize - vecSize/2)
        sinSquers = [(math.sin((i - dirSign*256)/1024*math.pi))**2
                    for i in range(0, vecSize)]

        minWeight = 0.2
        c = minWeight / (1 - minWeight)
        aWeights = [(w*(1-c) + c) / (1 + c) for w in sinSquers]
        bWeights = [((1 - w)*(1-c) + c) / (1 + c) for w in sinSquers]

        return aWeights, bWeights

    @staticmethod
    def calculatePosition(ca, cb, calibrationData):
        # pylint: disable=too-many-locals, too-many-statements
        aVec, bVec, aWeights, bWeights = calibrationData
        vecSize = len(aVec)

        def calcWrapAroundIndex(i):
            return i % vecSize

        def calcCost(i, a, b, aVec, bVec, *, enableWeights=False):
            aWeight = aWeights[i]
            bWeight = bWeights[i]
            if not enableWeights:
                aWeight = 1
                bWeight = 1

            tempA = aVec[i] - a
            tempA = tempA * tempA
            tempA *= aWeight

            tempB = bVec[i] - b
            tempB = tempB * tempB
            tempB *= bWeight

            return tempA + tempB

        sensor1Value = ca
        sensor2Value = cb

        stepSize = vecSize // 2

        bestI = 0
        bestCost = calcCost(bestI, sensor1Value, sensor2Value, aVec, bVec)

        i = bestI + stepSize
        while i < vecSize + bestI:
            cost = calcCost(calcWrapAroundIndex(i), sensor1Value, sensor2Value, aVec, bVec)

            if cost < bestCost:
                bestI = i
                bestCost = cost

            i += stepSize

        bestI = calcWrapAroundIndex(bestI)

        checkDir = 1

        while stepSize != 1:
            i = bestI

            stepSize = int(stepSize / 2)
            if stepSize == 0:
                stepSize = 1

            enableWeights = stepSize < vecSize//16

            i = calcWrapAroundIndex(i + stepSize * checkDir)

            cost = calcCost(i, sensor1Value, sensor2Value, aVec, bVec, enableWeights=enableWeights)

            if cost < bestCost:
                bestI = i
                bestCost = cost

                checkDir *= -1

                continue

            i = calcWrapAroundIndex(i - 2 * stepSize * checkDir)

            cost = calcCost(i, sensor1Value, sensor2Value, aVec, bVec, enableWeights=enableWeights)

            if cost < bestCost:
                bestI = i
                bestCost = cost

        return (bestI, bestCost)

    def getAdditionalDiagnostics(self):
        # pylint: disable=too-many-locals, too-many-statements
        vecSize = len(self.aVec)
        positions = []
        minCosts = []
        chA = []
        chB = []
        chADiffs = []
        chBDiffs = []

        aWeights, bWeights = OpticalEncoderDataVectorGenerator.calcSensorWeights(self.aVec, self.bVec)

        calibrationData = (self.aVec, self.bVec, aWeights, bWeights)
        for _, a, b in self.data:
            pos, cost = OpticalEncoderDataVectorGenerator.calculatePosition(
                    a, b, calibrationData)
            positions.append(pos)
            minCosts.append(cost / 16)
            chA.append(self.aVec[pos])
            chB.append(self.bVec[pos])
            chADiffs.append(a - self.aVec[pos])
            chBDiffs.append(b - self.bVec[pos])

        t = self.data[:, 0]
        velocities = [((p2 - p0 + vecSize//2) % vecSize - vecSize//2) / (t2 - t0)
                for p2, p0, t2, t0
                    in zip(positions[2:], positions[0:-2], t[2:], t[0:-2])]
        velocities = [0] + velocities + [0]

        return t, positions, velocities, minCosts, (chA, chB, chADiffs, chBDiffs)

    def showAdditionalDiagnosticPlots(self):
        fig = plt.figure(1)
        fig.suptitle('Full length sensor characteristic vector')
        x = np.arange(len(self.fullLengthAVector)) * len(self.aVec) / len(self.fullLengthAVector)
        plt.plot(x, self.fullLengthAVector, 'r+')
        plt.plot(x, self.fullLengthBVector, 'g+')
        plt.plot(self.aVec, 'm-')
        plt.plot(self.bVec, 'c-')

        t, positions, velocities, minCosts, (chA, chB, chADiffs, chBDiffs) = self.getAdditionalDiagnostics()

        fig = plt.figure(3)
        fig.suptitle('Encoder position')
        plt.plot(t, positions)

        fig = plt.figure(4)
        fig.suptitle('Min fitting cost')
        plt.plot(t, minCosts)

        fig = plt.figure(5)
        fig.suptitle('Encoder velocity')
        plt.plot(t, velocities)

        fig = plt.figure(6)
        fig.suptitle('Min fitting cost over encoder position')
        plt.plot(positions, minCosts, '+')

        fig = plt.figure(7)
        fig.suptitle('Velocity samples over encoder position')
        plt.plot(positions, velocities, '+', alpha=0.5)

        a = self.data[:,1]
        b = self.data[:,2]
        c = []
        for d in zip(a, b):
            c.append(math.sqrt((d[0]**2 + d[1]**2) / 2))
        fig = plt.figure(8)
        fig.suptitle('Sensor values (red and green) and expected values from characteristic vectors')
        plt.plot(a, 'r')
        plt.plot(b, 'g')
        plt.plot(chA, 'm')
        plt.plot(chB, 'c')

        fig = plt.figure(9)
        fig.suptitle('Diff between real sensor values and expected values')
        plt.plot(chADiffs, 'r')
        plt.plot(chBDiffs, 'g')

        fig = plt.figure(10)
        fig.suptitle('Diff between real sensor values and expected values over encoder position')
        plt.plot(positions, chADiffs, 'r+', alpha=0.5)
        plt.plot(positions, chBDiffs, 'g+', alpha=0.5)

        plt.show()

    def plotGeneratedVectors(self, box):
        fig = Figure(figsize=(5, 4), dpi=100)
        ax = fig.add_subplot()

        labelStr = ('Red is channel A and green is channel B. A good result\n'
                'is indicated by all points cohering to a smooth curve.')

        ax.plot(self.aVecShifted, 'r-')
        ax.plot(self.bVecShifted, 'g-')

        if self.oldAVec is not None and self.oldBVec is not None:
            ax.plot(self.oldAVec, 'm-')
            ax.plot(self.oldBVec, 'c-')

            labelStr += '\nMagenta and cyan curves are old channel A and channel B.'

        canvas = FigureCanvas(fig)
        canvas.set_size_request(600, 400)
        box.add(canvas)

        label = Gtk.Label(label=labelStr)
        label.set_use_markup(True)
        label.set_margin_start(30)
        label.set_margin_end(50)
        label.set_margin_top(8)
        label.set_margin_bottom(10)
        label.set_xalign(0.0)
        box.add(label)

        box.show_all()

    def writeVectorsToConfigFileString(self, configFileAsString, configClassName):
        configClassString = getConfigClassString(configFileAsString, configClassName)
        aVecPattern = OpticalEncoderDataVectorGenerator._aVecPattern
        bVecPattern = OpticalEncoderDataVectorGenerator._bVecPattern

        temp1 = aVecPattern.search(configClassString)
        temp2 = bVecPattern.search(configClassString)
        if temp1 is not None and temp2 is not None:
            configClassString = re.sub(aVecPattern, r'\g<beg>' + '2048' + r'\g<end>'
                    + intArrayToString(self.aVecShifted), configClassString)
            configClassString = re.sub(bVecPattern, r'\g<beg>' + '2048' + r'\g<end>'
                    + intArrayToString(self.bVecShifted), configClassString)

            configFileAsString = setConfigClassString(configFileAsString, configClassName, configClassString)
            return configFileAsString

        return ''

    def getGeneratedVectors(self):
        out = ''
        out += 'std::array<uint16_t, 2048 > aVec = ' + intArrayToString(self.aVecShifted) + '\n'
        out += 'std::array<uint16_t, 2048 > bVec = ' + intArrayToString(self.bVecShifted)

        return out

def createGuiBox(parent, nodeNr, getPortFun, configFilePath, configClassName):
    # pylint: disable=too-many-locals, too-many-statements
    calibrationBox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
    calibrationBox.set_margin_start(40)
    calibrationBox.set_margin_bottom(100)

    limitMovementButton = GuiFunctions.createToggleButton('Lock', getLowLev=True)
    limitMovementButton = (GuiFunctions.addTopLabelTo(
                '<b>Limit movement</b>\n Only move around locked position to avoid end limits',
                limitMovementButton[0]),
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

    pwmValue = 0
    pwmScale = GuiFunctions.creatHScale(pwmValue, 0, 1023, 10, getLowLev=True)
    pwmScale = (GuiFunctions.addTopLabelTo(
                '<b>Motor pwm value</b>\n Choose a value that results in a moderate constant velocity', pwmScale[0]),
            pwmScale[1])
    calibrationBox.pack_start(pwmScale[0], False, False, 0)

    testButton = GuiFunctions.createButton('Test pwm value', getLowLev=True)
    calibrationBox.pack_start(testButton[0], False, False, 0)
    startButton = GuiFunctions.createButton('Start calibration', getLowLev=True)
    calibrationBox.pack_start(startButton[0], False, False, 0)

    recordingProgressBar = GuiFunctions.creatProgressBar(label='Recording', getLowLev=True)
    analyzingProgressBar = GuiFunctions.creatProgressBar(label='Analyzing', getLowLev=True)
    statusLabel = GuiFunctions.createLabel('')

    threadMutex = threading.Lock()
    def updatePwmValue(widget):
        nonlocal pwmValue

        with threadMutex:
            pwmValue = widget.get_value()

    pwmScale[1].connect('value-changed', updatePwmValue)

    def resetGuiAfterCalibration():
        testButton[1].set_label('Test pwm value')
        testButton[1].set_sensitive(True)
        startButton[1].set_label('Start calibration')
        startButton[1].set_sensitive(True)
        calibrationBox.remove(recordingProgressBar[0])
        calibrationBox.remove(analyzingProgressBar[0])
        calibrationBox.remove(statusLabel)
        pwmScale[1].set_sensitive(True)
        limitMovementButton[1].set_sensitive(True)

    def updateStatusLabel(statusStr):
        statusLabel.set_label(statusStr)

    runThread = False
    def testPwmRun(nodeNr, port):
        # pylint: disable=too-many-statements
        nonlocal runThread

        try:
            with createServoManager(nodeNr, port, dt=0.0042) as servoManager:
                t = 0.0
                doneRunning = False
                pwmDir = 1
                moveDir = 0

                def sendCommandHandlerFunction(dt, servoManager):
                    nonlocal pwmDir
                    nonlocal moveDir

                    servo = servoManager.servoArray[0]

                    pwm = 0
                    with threadMutex:
                        pwm = pwmValue

                    pos = servo.getPosition(True)

                    if startPos is not None:
                        if pos - startPos < -1 and moveDir != -1:
                            moveDir = -1
                            pwmDir *= -1
                        elif pos - startPos > 1 and moveDir != 1:
                            moveDir = 1
                            pwmDir *= -1

                    if pwm != 0.0:
                        servo.setOpenLoopControlSignal(pwm * pwmDir, True)

                out = []

                def readResultHandlerFunction(dt, servoManager):
                    nonlocal t
                    nonlocal doneRunning

                    t += dt
                    servo = servoManager.servoArray[0]
                    opticalEncoderData = servo.getOpticalEncoderChannelData()
                    out.append([servo.getTime(),
                            opticalEncoderData.a,
                            opticalEncoderData.b,
                            opticalEncoderData.minCostIndex,
                            opticalEncoderData.minCost,
                            servo.getVelocity(),
                            servo.getPosition(True) / pi * 180.0])

                    GLib.idle_add(updateStatusLabel,
                            f'Sensor A: {opticalEncoderData.a}\n'
                            f'Sensor B: {opticalEncoderData.b}\n'
                            f'Encoder position: {opticalEncoderData.minCostIndex}\n'
                            f'Output Encoder position: {servo.getPosition(True) / pi * 180.0:0.2f}'
                    )

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
                        fig.suptitle('Sensor A and B values')
                        plt.plot(data[:, 0], data[:, 1], 'r')
                        plt.plot(data[:, 0], data[:, 2], 'g')

                        fig = plt.figure(2)
                        fig.suptitle('Encoder position')
                        plt.plot(data[:, 0], data[:, 3])
                        fig = plt.figure(3)
                        fig.suptitle('Velocity')
                        plt.plot(data[:, 0], data[:, 5], 'r')
                        fig = plt.figure(4)
                        fig.suptitle('Min fitting cost')
                        plt.plot(data[:, 0], data[:, 4])
                        fig = plt.figure(5)
                        fig.suptitle('Output encoder position')
                        plt.plot(data[:, 0], data[:, 6])
                        fig = plt.figure(6)
                        fig.suptitle('Min fitting cost over encoder position')
                        plt.plot(data[:, 3], data[:, 4], 'r-')
                        plt.plot(data[:, 3], data[:, 4], 'g+')
                        fig = plt.figure(7)
                        fig.suptitle('Velocity over encoder position')
                        plt.plot(data[:, 3], data[:, 5], 'r-')
                        plt.plot(data[:, 3], data[:, 5], 'g+')

                        fig = plt.figure(8)
                        fig.suptitle('Sensor values over encoder position')
                        plt.plot(data[:, 3], data[:, 1], 'r+')
                        plt.plot(data[:, 3], data[:, 2], 'g+')

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
            startButton[1].set_sensitive(False)
            limitMovementButton[1].set_sensitive(False)
            calibrationBox.pack_start(statusLabel, False, False, 0)

            calibrationBox.show_all()

            widget.set_label('Stop pwm test')
            with threadMutex:
                runThread = True
            testThread = threading.Thread(target=testPwmRun, args=(nodeNr, getPortFun(),))
            testThread.start()
        else:
            with threadMutex:
                runThread = False
            testThread.join()

    testButton[1].connect('clicked', onTestPwm)

    def updateRecordingProgressBar(fraction):
        recordingProgressBar[1].set_fraction(fraction)

    def updateAnalyzingProgressBar(fraction):
        analyzingProgressBar[1].set_fraction(fraction)

    def startCalibrationRun(nodeNr, port):
        # pylint: disable=too-many-locals, too-many-statements
        nonlocal runThread

        try:
            with createServoManager(nodeNr, port, dt=0.0042) as servoManager:

                def handleResults(opticalEncoderDataVectorGenerator):
                    dialog = Gtk.MessageDialog(
                            transient_for=parent,
                            flags=0,
                            message_type=Gtk.MessageType.INFO,
                            buttons=Gtk.ButtonsType.YES_NO,
                            text='Optical encoder calibration done!',
                    )
                    dialog.format_secondary_text(
                        "Do you want to plot extended data?"
                    )
                    response = dialog.run()
                    dialog.destroy()

                    if response == Gtk.ResponseType.YES:
                        opticalEncoderDataVectorGenerator.showAdditionalDiagnosticPlots()

                    dialog = Gtk.MessageDialog(
                            transient_for=parent,
                            flags=0,
                            message_type=Gtk.MessageType.INFO,
                            buttons=Gtk.ButtonsType.YES_NO,
                            text='Optical encoder calibration done!',
                    )
                    dialog.format_secondary_text(
                        "Should the configuration be updated with the new data?"
                    )
                    opticalEncoderDataVectorGenerator.plotGeneratedVectors(dialog.get_message_area())
                    dialog.get_widget_for_response(Gtk.ResponseType.YES).grab_focus()
                    response = dialog.run()
                    dialog.destroy()

                    if response == Gtk.ResponseType.YES:
                        with open(configFilePath, "r", encoding='utf-8') as configFile:
                            configFileAsString = configFile.read()
                            configFileAsString = opticalEncoderDataVectorGenerator.writeVectorsToConfigFileString(
                                    configFileAsString, configClassName)

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
                            "Please past in the new aVec and bVec manually"
                        )
                        box = dialog.get_message_area()
                        vecEntry = Gtk.Entry()
                        vecEntry.set_text(opticalEncoderDataVectorGenerator.getGeneratedVectors())
                        box.add(vecEntry)
                        box.show_all()
                        dialog.run()
                        dialog.destroy()

                def handleAnalyzeError(info):
                    dialog = Gtk.MessageDialog(
                            transient_for=parent,
                            flags=0,
                            message_type=Gtk.MessageType.ERROR,
                            buttons=Gtk.ButtonsType.OK,
                            text='Calibration failed during analyzing',
                    )
                    dialog.format_secondary_text(info)
                    dialog.run()
                    dialog.destroy()

                t = 0.0
                dirChangeWait = 0.0
                doneRunning = False
                pwmDir = 1
                moveDir = 0

                runTime = 110.0

                def sendCommandHandlerFunction(dt, servoManager):
                    nonlocal dirChangeWait
                    nonlocal pwmDir
                    nonlocal moveDir

                    servo = servoManager.servoArray[0]

                    pwm = 0
                    with threadMutex:
                        pwm = pwmValue

                    pos = servo.getPosition(True)

                    if startPos is not None:
                        if pos - startPos < -1:
                            dirChangeWait = 1.0
                            if moveDir != -1:
                                moveDir = -1
                                pwmDir *= -1
                        elif pos - startPos > 1:
                            dirChangeWait = 1.0
                            if moveDir != 1:
                                moveDir = 1
                                pwmDir *= -1

                    if t > (runTime - 10) * 0.5 and moveDir == 0:
                        moveDir = 1
                        pwmDir *= -1
                        dirChangeWait = 2.0

                    if pwm != 0.0:
                        servo.setOpenLoopControlSignal(pwm * pwmDir * min(1.0, 0.25 * t), True)

                out = []

                if os.path.isfile('optEncTestDataToLoad.txt') and t == 0.0:
                    dialog = Gtk.MessageDialog(
                            transient_for=parent,
                            flags=0,
                            message_type=Gtk.MessageType.INFO,
                            buttons=Gtk.ButtonsType.YES_NO,
                            text='Found file: "optEncTestDataToLoad.txt"!\n'
                                'Should this file be loaded instead of\n'
                                'running a new calibration?',
                    )
                    response = dialog.run()
                    dialog.destroy()

                    if response == Gtk.ResponseType.YES:
                        out = np.loadtxt('optEncTestDataToLoad.txt')
                        doneRunning = True

                def readResultHandlerFunction(dt, servoManager):
                    nonlocal t
                    nonlocal dirChangeWait
                    nonlocal doneRunning

                    servo = servoManager.servoArray[0]
                    opticalEncoderData = servo.getOpticalEncoderChannelData()
                    if t > 0.1 and dirChangeWait <= 0.0:
                        out.append([servo.getTime(),
                                opticalEncoderData.a,
                                opticalEncoderData.b,
                                opticalEncoderData.minCostIndex,
                                opticalEncoderData.minCost])

                    GLib.idle_add(updateRecordingProgressBar, t / runTime)

                    stop = t >= runTime
                    with threadMutex:
                        if runThread is False:
                            stop = True

                    if stop or parent.isClosed:
                        servoManager.removeHandlerFunctions()
                        doneRunning = True

                    if dirChangeWait > 0.0:
                        dirChangeWait -= dt
                    else:
                        t += dt

                servoManager.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction)

                while not doneRunning:
                    if not servoManager.isAlive():
                        runThread = False
                        break
                    time.sleep(0.1)

                data = np.array(out)

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

                if not shouldAbort():
                    np.savetxt('lastOptEncTestData.txt', data)
                    try:
                        configFileAsString = ''
                        with open(configFilePath, "r", encoding='utf-8') as configFile:
                            configFileAsString = configFile.read()

                        opticalEncoderDataVectorGenerator = OpticalEncoderDataVectorGenerator(
                                data, configFileAsString, configClassName, constVelIndex=4000,
                                shouldAbort=shouldAbort,
                                updateProgress=updateProgress)

                        GLib.idle_add(handleResults, opticalEncoderDataVectorGenerator)
                    except Exception as e:
                        GLib.idle_add(handleAnalyzeError, format(e))

        except Exception as e:
            GuiFunctions.exceptionMessage(parent, e)
        finally:
            GLib.idle_add(resetGuiAfterCalibration)

    def onStartCalibration(widget):
        nonlocal testThread
        nonlocal runThread

        if widget.get_label() == 'Start calibration':
            widget.set_label('Abort calibration')

            recordingProgressBar[1].set_fraction(0.0)
            analyzingProgressBar[1].set_fraction(0.0)
            testButton[1].set_sensitive(False)
            calibrationBox.pack_start(recordingProgressBar[0], False, False, 0)
            calibrationBox.pack_start(analyzingProgressBar[0], False, False, 0)
            pwmScale[1].set_sensitive(False)
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

    startButton[1].connect('clicked', onStartCalibration)

    calibrationBox.show_all()
    return calibrationBox
