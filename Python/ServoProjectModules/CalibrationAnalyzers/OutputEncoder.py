'''
Module for calibrating the output encoder
'''
# pylint: disable=duplicate-code

import os
from ServoProjectModules.CalibrationAnalyzers.Helper import *  # pylint: disable=wildcard-import, unused-wildcard-import

class OutputEncoderCalibrationGenerator:
    def __init__(self, data, wrapAround, unitsPerRev):
        # pylint: disable=too-many-locals, too-many-branches, too-many-statements
        unitsPerRev = abs(unitsPerRev)
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

        lastPos = int(round((self.data[0, 1] % 4096) / 8))
        inSequence = []
        for d in self.data:
            if d[0] < 1.0:
                continue
            if d[1] != maxPos and d[1] != minPos:
                pos = int(round((d[1] % 4096) / 8))
                if pos == posListSize:
                    pos = posListSize - 1

                if lastPos == pos:
                    inSequence.append(d[2])
                elif len(inSequence) > 0:
                    posList[lastPos].append([sum(inSequence), len(inSequence)])
                    inSequence = []
                lastPos = pos

        self.meanList = []
        for d in posList:
            if len(d) < 2:
                self.meanList.append(None)
            else:
                d.sort(key=lambda dd: -dd[1])
                listOfMeans = [dd[0] / dd[1] for dd in d]
                mean = sum(listOfMeans) / len(listOfMeans)
                eps = 0.0001
                underMean = [dd for dd in listOfMeans if dd < mean - eps]
                overMean = [dd for dd in listOfMeans if dd > mean + eps]
                l = min(len(underMean), len(overMean))
                if l != 0:
                    self.meanList.append((sum(underMean[0:l]) + sum(overMean[0:l])) / (2 * l))
                elif len(underMean) == 0 and len(overMean) == 0:
                    self.meanList.append(mean)

        i = 0
        if wrapAround:
            while self.meanList[i - 1] is None:
                i -= 1

        noneSegList = []
        end = i + len(self.meanList)
        while True:
            noneSeg = []
            while self.meanList[i] is not None:
                i += 1
                if i == end:
                    break
            if i == end:
                break
            while self.meanList[i] is None:
                noneSeg.append(i)
                i += 1
                if i == end:
                    break
            noneSegList.append(noneSeg)
            if i == end:
                break

        if len(noneSegList) == 1 and len(noneSegList[0]) == len(self.meanList):
            return

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

    @staticmethod
    def checkForInvertedEncoder(data):
        minPos = min(data[:, 1])
        maxPos = max(data[:, 1])
        minDiff = min(data[:, 2])
        maxDiff = max(data[:, 2])

        meanCompSlope = abs(maxDiff - minDiff) / abs(maxPos - minPos)

        return meanCompSlope > 1.5

    def isInverted(self):
        return OutputEncoderCalibrationGenerator.checkForInvertedEncoder(self.data)

    _compVecPattern = re.compile(
            r'(?P<beg>.*createOutputEncoderHandler\(\)\s*\{(.*\n)*?\s*(constexpr)?\s+(static)?\s+'
            r'std\s*::\s*array\s*<\s*int16_t\s*,\s*513\s*>\s*compVec\s*=\s*)\{\s*(?P<vec>[^\}]+)\s*\};')

    @staticmethod
    def checkForPreviousCalibration(configFileAsString, configClassName):
        configClassString = getConfigClassString(configFileAsString, configClassName)

        temp = OutputEncoderCalibrationGenerator._compVecPattern.search(configClassString)
        if temp is not None:
            if temp.group('vec') != '0':
                return True

        return False

    @staticmethod
    def resetPreviousCalibration(configFileAsString, configClassName):
        configClassString = getConfigClassString(configFileAsString, configClassName)
        configClassString = re.sub(OutputEncoderCalibrationGenerator._compVecPattern,
                r'\g<beg>{0};', configClassString)
        configFileAsString = setConfigClassString(configFileAsString, configClassName, configClassString)

        return configFileAsString

    def plotGeneratedVector(self, box):
        fig = Figure(figsize=(5, 4), dpi=100)
        ax = fig.add_subplot()

        x = range(0, 4096 + 8, 8)
        ax.plot(self.data[:, 1] % 4096, self.data[:, 2], 'b+', alpha=0.25)
        ax.plot(x, self.meanList, 'g-+')

        canvas = FigureCanvas(fig)
        canvas.set_size_request(600, 400)
        zoomWindow = 256
        defaultXLims = [-x[-1] * 0.02, x[-1] * 1.02]
        xLims = defaultXLims
        ax.set_xlim(xLims[0], xLims[1])

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

        box.show_all()

    def writeVectorToConfigFileString(self, configFileAsString, configClassName):
        configClassString = getConfigClassString(configFileAsString, configClassName)

        temp = OutputEncoderCalibrationGenerator._compVecPattern.search(configClassString)
        if temp is not None:
            configClassString = re.sub(OutputEncoderCalibrationGenerator._compVecPattern, r'\g<beg>' +
                    intArrayToString(self.meanList), configClassString)

            configFileAsString = setConfigClassString(configFileAsString, configClassName, configClassString)
            return configFileAsString

        return ''

    def getGeneratedVector(self):
        out = ''
        out += 'std::array<int16_t, 513 > compVec = ' + intArrayToString(self.meanList)

        return out

    def invertOutputEncoder(self, configFileAsString, configClassName):
        configClassString = getConfigClassString(configFileAsString, configClassName)

        temp = outputEncoderUnitPerRevPattern.search(configClassString)

        if temp is not None:
            unitsPerRevStr = temp.group('units')
            unitsPerRevStr = '-(' + unitsPerRevStr + ')'

            while unitsPerRevStr.find('-(-(') == 0:
                unitsPerRevStr = unitsPerRevStr[4:-2]

            configClassString = re.sub(outputEncoderUnitPerRevPattern,
                    r'\g<beg>' + r'\g<encoderType>' + r'\g<mid>' + unitsPerRevStr + r'\g<end>', configClassString)

            configFileAsString = setConfigClassString(configFileAsString, configClassName, configClassString)
            return configFileAsString

        return ''

def createGuiBox(parent, nodeNr, getPortFun, configFilePath, configClassName):
    # pylint: disable=too-many-locals, too-many-statements
    calibrationBox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
    calibrationBox.set_margin_start(40)
    calibrationBox.set_margin_bottom(100)

    contorlParameters = ControlParameters(14, 14 * 4, 14 * 32, 1.0)

    controlSpeedScale = GuiFunctions.creatHScale(contorlParameters.getMainSpeed(), 0, 100, 1, getLowLev=True)
    controlSpeedScale = GuiFunctions.addTopLabelTo('<b>Control speed</b>\n'
            ' Higher value results in tighter control but increases noise feedback\n'
            '(control theory: pole placement of slowest pole)', controlSpeedScale[0]), controlSpeedScale[1]
    calibrationBox.pack_start(controlSpeedScale[0], False, False, 0)

    advancedParametersButton = GuiFunctions.createButton('Set advanced parameters', getLowLev=True)
    calibrationBox.pack_start(advancedParametersButton[0], False, False, 0)

    def onControlSpeedScaleChange(widget):
        contorlParameters.setMainSpeed(controlSpeedScale[1].get_value())

    controlSpeedScale[1].connect('value-changed', onControlSpeedScaleChange)

    def onAdvancedParamClicked(widget):
        nonlocal contorlParameters
        contorlParameters = GuiFunctions.openAdvancedParametersDialog(parent, contorlParameters)
        controlSpeedScale[1].set_value(contorlParameters.getMainSpeed())

    advancedParametersButton[1].connect('clicked', onAdvancedParamClicked)

    startButton = GuiFunctions.createButton('Start calibration', getLowLev=True)

    recordingProgressBar = GuiFunctions.creatProgressBar(label='Recording', getLowLev=True)
    directionLabel = GuiFunctions.createLabel('')
    directionLabel.set_margin_start(50)

    threadMutex = threading.Lock()
    testThread = None

    changeTorqueDirTime = 6.0
    runTime = 120.0 + changeTorqueDirTime

    def resetGuiAfterCalibration():
        controlSpeedScale[1].set_sensitive(True)
        advancedParametersButton[1].set_sensitive(True)
        startButton[1].set_label('Start calibration')
        startButton[1].set_sensitive(True)
        calibrationBox.remove(recordingProgressBar[0])
        calibrationBox.remove(directionLabel)

    runThread = False

    def updateRecordingProgressBar(fraction, pos):
        timeBeforeDirChange = (runTime - changeTorqueDirTime) / 2
        timeAfterDirChange = timeBeforeDirChange + changeTorqueDirTime
        stepsStr = []
        stepsStr.append('1) Move the servo-output-shaft over its range of motion.\n')
        stepsStr.append('2) Leave it somewhere in the middle.\n')
        stepsStr.append(f'3) Apply constant torque in CW direction ({timeBeforeDirChange:.0f} sec).\n')
        stepsStr.append(f'4) Change torque direction to CCW ({changeTorqueDirTime:.0f} sec).\n')
        stepsStr.append(f'5) Apply constant torque in CCW direction ({timeBeforeDirChange:.0f} sec).\n')
        if fraction < 0:
            if fraction <= -1:
                recordingProgressBar[1].set_text('Waiting for move to complete')

                stepsStr[0] = '<b>' + stepsStr[0] + '</b>\n'
            else:
                recordingProgressBar[1].set_text('Confirming end positions')

                stepsStr[1] = '<b>' + stepsStr[1] + '</b>\n'

            fraction = -fraction
        else:
            currentRunTime = fraction * runTime
            if currentRunTime < timeBeforeDirChange:
                fraction = currentRunTime / timeBeforeDirChange
                recordingProgressBar[1].set_text('Recording with CW torque')

                stepsStr[2] = '<b>' + stepsStr[2] + '</b>\n'
            elif currentRunTime < timeAfterDirChange:
                fraction = (currentRunTime - timeBeforeDirChange) / changeTorqueDirTime
                fraction = 1.0 - fraction
                recordingProgressBar[1].set_text('Change to CCW torque')

                stepsStr[3] = '<b>' + stepsStr[3] + '</b>\n'
            else:
                fraction = (currentRunTime - timeAfterDirChange) / (runTime - timeAfterDirChange)
                recordingProgressBar[1].set_text('Recording with CCW torque')

                stepsStr[4] = '<b>' + stepsStr[4] + '</b>\n'

        directionLabel.set_text(''.join(stepsStr) + '\nRaw encoder position is ' + str(int(pos)))
        directionLabel.set_use_markup(True)
        recordingProgressBar[1].set_fraction(fraction)

    def handleResults(data):
        with open(configFilePath, 'r', encoding='utf-8') as configFile:
            configFileAsString = configFile.read()

            magneticEncoder, unitsPerRev = getConfiguredOutputEncoderData(
                getConfigClassString(configFileAsString, configClassName))
            outputEncoderCalibrationGenerator = OutputEncoderCalibrationGenerator(data, magneticEncoder, unitsPerRev)

            if outputEncoderCalibrationGenerator.isInverted():
                dialog = Gtk.MessageDialog(
                        transient_for=parent,
                        flags=0,
                        message_type=Gtk.MessageType.INFO,
                        buttons=Gtk.ButtonsType.YES_NO,
                        text='Output encoder needs to be inverted',
                )
                dialog.format_secondary_text(
                    'Should the output encoder be inverted in the configuration?'
                )
                response = dialog.run()
                dialog.destroy()

                if response == Gtk.ResponseType.YES:
                    configFileAsString = outputEncoderCalibrationGenerator.invertOutputEncoder(
                            configFileAsString, configClassName)

                    if configFileAsString != '':
                        with open(configFilePath, 'w', encoding='utf-8') as configFile:
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
                        'Please invert the output encoder manually'
                    )
                    response = dialog.run()
                    dialog.destroy()

                    return

            dialog = Gtk.MessageDialog(
                    transient_for=parent,
                    flags=0,
                    message_type=Gtk.MessageType.INFO,
                    buttons=Gtk.ButtonsType.YES_NO,
                    text='Output encoder calibration done!',
            )
            dialog.format_secondary_text(
                'Should the configuration be updated with the green compensation vector?'
            )
            outputEncoderCalibrationGenerator.plotGeneratedVector(dialog.get_message_area())
            dialog.get_widget_for_response(Gtk.ResponseType.YES).grab_focus()
            response = dialog.run()
            dialog.destroy()

            if response == Gtk.ResponseType.YES:
                configFileAsString = outputEncoderCalibrationGenerator.writeVectorToConfigFileString(
                        configFileAsString, configClassName)

                if configFileAsString != '':
                    with open(configFilePath, 'w', encoding='utf-8') as configFile:
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
                    'Please past in the new compensation vector manually'
                )
                box = dialog.get_message_area()
                vecEntry = Gtk.Entry()
                vecEntry.set_text(outputEncoderCalibrationGenerator.getGeneratedVector())
                box.add(vecEntry)
                box.show_all()
                response = dialog.run()
                dialog.destroy()

    simulationConfig = False

    def startCalibrationRun(nodeNr, port, loadDataFileInstead):
        # pylint: disable=too-many-locals, too-many-statements
        nonlocal runThread

        try:
            controlSpeedScale[1].set_sensitive(False)
            advancedParametersButton[1].set_sensitive(False)

            def initFun(servoArray):
                controlSpeed, velControlSpeed, filterSpeed, inertiaMarg = contorlParameters.getValues()
                servoArray[0].setControlSpeed(controlSpeed, velControlSpeed, filterSpeed, inertiaMarg)
                servoArray[0].setBacklashControlSpeed(0.0, 3.0, 0.0)

            with createServoManager(nodeNr, port, dt=0.003, initFunction=initFun) as servoManager:
                t = -6.2
                doneRunning = False
                refPos = 0.0
                minPos = None
                maxPos = None
                direction = 1

                out = []

                encPos = None
                filteredVel = 0.0

                if simulationConfig or port == '':
                    servo = servoManager.servoArray[0]
                    p = servo.getPosition(True)
                    minPos = p - 1.5
                    maxPos = p + 1.5
                    out.append([t, (minPos - servo.getOffset()) / servo.getScaling(), 0.0 / servo.getScaling()])
                    out.append([t, (maxPos - servo.getOffset()) / servo.getScaling(), 0.0 / servo.getScaling()])

                if loadDataFileInstead:
                    out = np.loadtxt('outputEncoderDataToLoad.txt')
                    doneRunning = True

                def sendCommandHandlerFunction(dt, servoManager):
                    nonlocal t
                    nonlocal refPos
                    nonlocal minPos
                    nonlocal maxPos
                    nonlocal direction

                    servo = servoManager.servoArray[0]

                    if t < 0:
                        servo.setOpenLoopControlSignal(0, True)
                    else:
                        refVel = (maxPos - minPos) / (runTime - changeTorqueDirTime) * 6

                        if abs(t - runTime / 2) < changeTorqueDirTime / 2:
                            refVel = 0.0

                        refPos += direction * refVel * dt

                        if refPos >= maxPos:
                            refPos = maxPos
                            direction = -1
                        elif refPos <= minPos:
                            refPos = minPos
                            direction = 1

                        servo.setReference(refPos, direction * refVel, 0.0)

                def readResultHandlerFunction(dt, servoManager):
                    # pylint: disable=too-many-locals, too-many-branches, too-many-statements
                    nonlocal t
                    nonlocal doneRunning
                    nonlocal encPos
                    nonlocal refPos
                    nonlocal minPos
                    nonlocal maxPos
                    nonlocal filteredVel
                    nonlocal out

                    servo = servoManager.servoArray[0]

                    newMotorPos = servo.getPosition(False)
                    newEncPos = servo.getPosition(True)
                    newVel = servo.getVelocity()

                    rawEncPos = (newEncPos - servo.getOffset()) / servo.getScaling()

                    out.append([t,
                            rawEncPos,
                            (newEncPos - newMotorPos) / servo.getScaling()])

                    if t < 0.0:
                        if encPos is None:
                            encPos = newEncPos
                        else:
                            noiseLevel = abs(abs(newEncPos - encPos) - abs(newVel * dt))
                            encPos = newEncPos
                            filteredVel = 0.95 * filteredVel + 0.05 * newVel

                            if minPos is None:
                                minPos = encPos
                                maxPos = encPos
                            elif encPos < minPos:
                                t = -6.1
                                minPos = encPos
                            elif encPos > maxPos:
                                t = -6.1
                                maxPos = encPos
                            elif abs(maxPos - minPos) < 0.1:
                                t = -6.1
                            elif ((encPos - minPos) / (maxPos - minPos) < 0.1 or
                                    (encPos - minPos) / (maxPos - minPos) > 0.9):
                                t = -6.1
                            elif noiseLevel > 0.02:
                                out = []
                                minPos = encPos
                                maxPos = encPos
                                t = -6.1
                            elif abs(filteredVel) > 0.1:
                                t = -6.1
                            else:
                                t += dt

                            if t >= 0:
                                data = np.array(out)
                                if OutputEncoderCalibrationGenerator.checkForInvertedEncoder(data):
                                    t = runTime
                                else:
                                    t = 0.0
                                    out = []
                                    refPos = encPos

                        GLib.idle_add(updateRecordingProgressBar, t / 6.0, round(rawEncPos))
                    else:
                        t += dt

                        GLib.idle_add(updateRecordingProgressBar, t / runTime, round(rawEncPos))

                    stop = t > runTime
                    with threadMutex:
                        if runThread is False:
                            stop = True
                    if stop or parent.isClosed:
                        servoManager.removeHandlerFunctions()
                        doneRunning = True
                        return

                servoManager.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction)

                while not doneRunning:
                    if not servoManager.isAlive():
                        runThread = False
                        break
                    time.sleep(0.1)

                servoManager.shutdown()

                if runThread is True:
                    data = np.array(out)
                    np.savetxt('lastOutputEncoderData.txt', data)
                    GLib.idle_add(handleResults, data)

        except Exception as e:
            GuiFunctions.exceptionMessage(parent, e)
        finally:
            GLib.idle_add(resetGuiAfterCalibration)

    def onStartCalibration(widget):
        nonlocal testThread
        nonlocal runThread
        nonlocal simulationConfig

        if widget.get_label() == 'Start calibration':
            with open(configFilePath, 'r', encoding='utf-8') as configFile:
                configFileAsString = configFile.read()

                if OutputEncoderCalibrationGenerator.checkForPreviousCalibration(configFileAsString, configClassName):
                    dialog = Gtk.MessageDialog(
                            transient_for=parent,
                            flags=0,
                            message_type=Gtk.MessageType.ERROR,
                            buttons=Gtk.ButtonsType.YES_NO,
                            text='Output encoder calibration already done for this configuration!',
                    )
                    dialog.format_secondary_text(
                        'Output encoder calibration only works on configurations without previous calibration.\n\n'
                        'Should the calibration be reset?'
                    )
                    response = dialog.run()
                    dialog.destroy()

                    if response == Gtk.ResponseType.NO:
                        return

                    configFileAsString = OutputEncoderCalibrationGenerator.resetPreviousCalibration(
                            configFileAsString, configClassName)
                    with open(configFilePath, 'w', encoding='utf-8') as configFile:
                        configFile.write(configFileAsString)
                        GuiFunctions.transferToTargetMessage(parent)

                    return

                simulationConfig = isSimulationConfig(
                        getConfigClassString(configFileAsString, configClassName))

            widget.set_label('Abort calibration')

            recordingProgressBar[1].set_fraction(1.0)
            calibrationBox.pack_start(recordingProgressBar[0], False, False, 0)
            directionLabel.set_text('')
            calibrationBox.pack_start(directionLabel, False, False, 0)

            calibrationBox.show_all()

            loadDataFileInstead = False
            if os.path.isfile('outputEncoderDataToLoad.txt'):
                dialog = Gtk.MessageDialog(
                        transient_for=parent,
                        flags=0,
                        message_type=Gtk.MessageType.INFO,
                        buttons=Gtk.ButtonsType.YES_NO,
                        text='Found file: "outputEncoderDataToLoad.txt"!\n'
                            'Should this file be loaded instead of\n'
                            'running a new calibration?',
                )
                response = dialog.run()
                dialog.destroy()

                loadDataFileInstead = response == Gtk.ResponseType.YES

            with threadMutex:
                runThread = True
            testThread = threading.Thread(target=startCalibrationRun, args=(nodeNr, getPortFun(), loadDataFileInstead,))
            testThread.start()
        else:
            with threadMutex:
                runThread = False
            testThread.join()

    startButton[1].connect('clicked', onStartCalibration)
    calibrationBox.pack_start(startButton[0], False, False, 0)
    calibrationBox.show_all()

    return calibrationBox
