'''
Module for calibrating motor cogging torque
'''
# pylint: disable=duplicate-code

import os
from ServoProjectModules.CalibrationAnalyzers.Helper import *  # pylint: disable=wildcard-import, unused-wildcard-import

class CoggingTorqueCalibrationGenerator:
    _posForcePattern = re.compile(
            r'(?P<beg>.*getPosDepForceCompVec\(\)\s*\{(.*\n)*?\s*(constexpr)?\s+(static)?\s+std\s*::\s*array\s*'
            r'<\s*int16_t\s*,\s*512\s*>\s+vec\s*=\s*)\{\s*(?P<vec>[^\}]*)\s*\};')
    _posFricPattern = re.compile(
            r'(?P<beg>.*getPosDepFrictionCompVec\(\)\s*\{(.*\n)*?\s*(constexpr)?\s+(static)?\s+std\s*::\s*array\s*'
            r'<\s*int16_t\s*,\s*512\s*>\s+vec\s*=\s*)\{\s*(?P<vec>[^\}]*)\s*\};')


    def __init__(self, data, configFileAsString='', configClassName='', *, posRes=32):
        # pylint: disable=too-many-locals, too-many-statements
        self.data = data

        self.positions = np.array(data[:, 6])
        self.forces = np.array(data[:,5])

        self.posRes = int(round(posRes))

        samplesList = []
        for i in range (0, 2048//self.posRes):
            samplesList.append([])

        for d in zip(self.positions, self.forces):
            i = int(round(d[0] / self.posRes)) % len(samplesList)
            samplesList[i].append(d[1])

        self.samplesList = [sorted(samples) for samples in samplesList]
        lowCogging = [samples[len(samples)//8:len(samples)*3//8] for samples in self.samplesList]
        highCogging = [samples[-len(samples)*3//8:-len(samples)//8] for samples in self.samplesList]

        def combineLists(listOfLists):
            out = []
            for l in listOfLists:
                out += l

            return out

        lowAvg = np.mean(combineLists(lowCogging))
        highAvg = np.mean(combineLists(highCogging))

        lowCoggingZeroMean = [[v - lowAvg for v in samples] for samples in lowCogging]
        highCoggingZeroMean = [[v - highAvg for v in samples] for samples in highCogging]

        coggingData = [(i * self.posRes,
                    (lowSamp, highSamp) if np.mean(lowSamp+highSamp) >= 0.0 else (highSamp, lowSamp))
            for i, (lowSamp, highSamp) in
                enumerate(zip(lowCoggingZeroMean, highCoggingZeroMean))]

        x = [p for p, _ in coggingData]
        outputX = np.arange(0, 2048, 4)
        ff = np.fft.fftfreq(len(x), (x[1]-x[0]))

        y = np.array([calcSpikeResistantAvarage(l, excluded=0.25) for _, (l, _) in coggingData])
        self.lowCoggingFiltered = fftFilter(x, y, max(ff), upSampleTt=outputX) + lowAvg

        y = np.array([calcSpikeResistantAvarage(l, excluded=0.25) for _, (_, l) in coggingData])
        self.highCoggingFiltered = fftFilter(x, y, max(ff), upSampleTt=outputX) + highAvg

        self.cogging = (self.lowCoggingFiltered + self.highCoggingFiltered) / 2


        self.lowFriction = [self.cogging[i*(self.posRes//4)] - np.array(s) for i, s in enumerate(lowCogging)]
        self.highFriction = [np.array(s) - self.cogging[i*(self.posRes//4)] for i, s in enumerate(highCogging)]

        y = np.array([calcSpikeResistantAvarage(list(ls) + list(hs), excluded=0.25)
                for ls, hs in zip(self.lowFriction, self.highFriction)])
        self.friction = fftFilter(x, y, max(ff), upSampleTt=outputX)

        self.oldCogging = None
        self.oldFriction = None

        configClassString = getConfigClassString(configFileAsString, configClassName)

        if configClassString != '':
            temp1 = CoggingTorqueCalibrationGenerator._posForcePattern.search(configClassString)
            temp2 = CoggingTorqueCalibrationGenerator._posFricPattern.search(configClassString)

            if temp1 is not None and temp2 is not None:
                oldCoggingStr = temp1.group('vec')
                oldFrictionStr = temp2.group('vec')
                oldCoggingStr = '[' + oldCoggingStr + ']'
                oldFrictionStr = '[' + oldFrictionStr + ']'

                self.oldCogging = np.array(eval(oldCoggingStr))  # pylint: disable=eval-used
                self.oldFriction = np.array(eval(oldFrictionStr))  # pylint: disable=eval-used

                if len(self.oldCogging) <= 1:
                    self.oldCogging = None
                if len(self.oldFriction) <= 1:
                    self.oldFriction = None

    @staticmethod
    def checkForPreviousCalibration(configFileAsString, configClassName):
        configClassString = getConfigClassString(configFileAsString, configClassName)

        temp = CoggingTorqueCalibrationGenerator._posForcePattern.search(configClassString)
        if temp is not None:
            if temp.group('vec') != '0':
                return True

        temp = CoggingTorqueCalibrationGenerator._posFricPattern.search(configClassString)
        if temp is not None:
            if temp.group('vec') != '0':
                return True

        return False

    @staticmethod
    def resetPreviousCalibration(configFileAsString, configClassName):
        configClassString = getConfigClassString(configFileAsString, configClassName)
        configClassString = re.sub(CoggingTorqueCalibrationGenerator._posForcePattern,
                r'\g<beg>{0};', configClassString)
        configClassString = re.sub(CoggingTorqueCalibrationGenerator._posFricPattern,
                r'\g<beg>{0};', configClassString)
        configFileAsString = setConfigClassString(configFileAsString, configClassName, configClassString)

        return configFileAsString

    def showAdditionalDiagnosticPlots(self):
        data = self.data

        t = data[:, 0] - data[0, 0]
        fig = plt.figure(1)
        fig.suptitle('Position')
        plt.plot(t, data[:, 1], 'g')

        fig = plt.figure(2)
        fig.suptitle('Velocity')
        plt.plot(t, data[:, 2])

        fig = plt.figure(3)
        fig.suptitle('Motor pos diff over motor position')
        plt.plot(self.positions[0:-1],
                [(d1 - d0 + 1024)%2048-1024 for d1, d0 in zip(self.positions[1:], self.positions[0:-1])], '+')

        fig = plt.figure(4)
        fig.suptitle('Error at motor')
        plt.plot(t, data[:, 4])

        fig = plt.figure(5)
        fig.suptitle('Control signal')
        plt.plot(t, data[:, 5])

        fig = plt.figure(6)
        fig.suptitle('Cogging over motor position')

        lowCogging = [samples[0:len(samples)//2] for samples in self.samplesList]
        highCogging = [samples[-len(samples)//2:] for samples in self.samplesList]

        x = np.arange(0, 2048, self.posRes)
        for p, ls, hs in zip(x, lowCogging, highCogging):
            plt.plot([p]*len(ls), ls, 'b+', alpha=0.2)
            plt.plot([p]*len(hs), hs, 'r+', alpha=0.2)

        x = np.arange(0, 2048, 4)
        plt.plot(x, self.lowCoggingFiltered, 'b')
        plt.plot(x, self.highCoggingFiltered, 'r')
        if self.oldCogging is not None:
            plt.plot(x, self.oldCogging, 'c')
        plt.plot(x, self.cogging, 'g')

        fig = plt.figure(7)
        fig.suptitle('Friction over motor position')

        x = np.arange(0, 2048, self.posRes)
        for p, ls, hs in zip(x, self.lowFriction, self.highFriction):
            plt.plot([p]*len(ls), ls, 'b+', alpha=0.2)
            plt.plot([p]*len(hs), hs, 'r+', alpha=0.2)

        x = np.arange(0, 2048, 4)
        if self.oldFriction is not None:
            plt.plot(x, self.oldFriction, 'c')
        plt.plot(x, self.friction, 'g')

        plt.show()

    def plotGeneratedVector(self, box):
        fig = Figure(figsize=(5, 4), dpi=100)
        ax = fig.add_subplot()

        x = np.arange(0, 2048, self.posRes)

        lowCogging = [samples[0:len(samples)//2] for samples in self.samplesList]
        highCogging = [samples[-len(samples)//2:] for samples in self.samplesList]

        for p, ls, hs in zip(x, lowCogging, highCogging):
            ax.plot([p]*len(ls), ls, 'b+', alpha=0.2)
            ax.plot([p]*len(hs), hs, 'r+', alpha=0.2)

        x = np.arange(0, 2048, 4)
        if self.oldCogging is not None and self.oldFriction is not None:
            ax.plot(x, self.oldCogging - self.oldFriction, 'c+')
            ax.plot(x, self.oldCogging + self.oldFriction, 'c+')
        ax.plot(x, self.cogging - self.friction, 'g')
        ax.plot(x, self.cogging + self.friction, 'g')

        canvas = FigureCanvas(fig)
        canvas.set_size_request(600, 400)
        box.add(canvas)

        box.show_all()

    def writeVectorToConfigFileString(self, configFileAsString, configClassName):
        configClassString = getConfigClassString(configFileAsString, configClassName)

        temp = True
        temp = CoggingTorqueCalibrationGenerator._posForcePattern.search(configClassString) is not None and temp
        temp = CoggingTorqueCalibrationGenerator._posFricPattern.search(configClassString) is not None and temp
        if temp:
            configClassString = re.sub(CoggingTorqueCalibrationGenerator._posForcePattern,
                    r'\g<beg>' + intArrayToString(self.cogging), configClassString)
            configClassString = re.sub(CoggingTorqueCalibrationGenerator._posFricPattern,
                    r'\g<beg>' + intArrayToString(self.friction), configClassString)

            configFileAsString = setConfigClassString(configFileAsString, configClassName, configClassString)
            return configFileAsString

        return ''

    def getGeneratedVectors(self):
        out = ''
        out += 'constexpr static std::array<int16_t, 512> posDepForceCompVec = ' + intArrayToString(self.cogging)
        out += '\nconstexpr static std::array<int16_t, 512> posDepFrictionCompVec = ' + intArrayToString(self.friction)

        return out

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

    resetCalibrationButton = GuiFunctions.createButton('Reset calibration', getLowLev=True)

    def onResetCalibration(widget):
        try:
            with open(configFilePath, "r", encoding='utf-8') as configFile:
                configFileAsString = configFile.read()
                if not CoggingTorqueCalibrationGenerator.checkForPreviousCalibration(
                        configFileAsString, configClassName):
                    return

                configFileAsString = CoggingTorqueCalibrationGenerator.resetPreviousCalibration(
                        configFileAsString, configClassName)
                with open(configFilePath, "w", encoding='utf-8') as configFile:
                    configFile.write(configFileAsString)
                    GuiFunctions.transferToTargetMessage(parent)
        except Exception as e:
            GuiFunctions.exceptionMessage(parent, e)

    calibrationBox.pack_start(resetCalibrationButton[0], False, False, 0)

    resetCalibrationButton[1].connect('clicked', onResetCalibration)

    calibrationLevelOptions = ('Standard (~3 min)', 'Fine (~5 min)', 'Ultra (~11 min)')
    calibrationLevelComboBox = GuiFunctions.creatComboBox(calibrationLevelOptions[0], 
            calibrationLevelOptions, getLowLev=True)
    calibrationLevelComboBox = (GuiFunctions.addTopLabelTo('<b>Position resolution</b>', calibrationLevelComboBox[0]),
                            calibrationLevelComboBox[1])
    calibrationBox.pack_start(calibrationLevelComboBox[0], False, False, 0)

    startButton = GuiFunctions.createButton('Start', getLowLev=True)

    recordingProgressBar = GuiFunctions.creatProgressBar(label='Recording', getLowLev=True)

    calibrationBox.pack_start(startButton[0], False, False, 0)
    calibrationBox.show_all()

    threadMutex = threading.Lock()

    def updateProgress(fraction):
        recordingProgressBar[1].set_fraction(fraction)

    def resetGuiAfterCalibration():
        startButton[1].set_label('Start')
        startButton[1].set_sensitive(True)
        controlSpeedScale[1].set_sensitive(True)
        advancedParametersButton[1].set_sensitive(True)
        resetCalibrationButton[1].set_sensitive(True)
        calibrationBox.remove(recordingProgressBar[0])

    runThread = False

    def handleResults(data):
        configFileAsString = ''
        with open(configFilePath, "r", encoding='utf-8') as configFile:
            configFileAsString = configFile.read()

        posRes = 32 / (2**calibrationLevelComboBox[1].get_active())

        coggingTorqueCalibrationGenerator = CoggingTorqueCalibrationGenerator(data,
                configFileAsString, configClassName, posRes=posRes)

        dialog = Gtk.MessageDialog(
                transient_for=parent,
                flags=0,
                message_type=Gtk.MessageType.INFO,
                buttons=Gtk.ButtonsType.YES_NO,
                text='Test done',
        )
        dialog.format_secondary_text(
            "Do you want to plot the recorded data?"
        )
        response = dialog.run()
        dialog.destroy()

        if response == Gtk.ResponseType.YES:
            coggingTorqueCalibrationGenerator.showAdditionalDiagnosticPlots()

            dialog = Gtk.MessageDialog(
                    transient_for=parent,
                    flags=0,
                    message_type=Gtk.MessageType.INFO,
                    buttons=Gtk.ButtonsType.YES_NO,
                    text='Cogging torque calibration done!',
            )
            dialog.format_secondary_text(
                "Should the configuration be updated with the green compensation vectors?"
            )
            coggingTorqueCalibrationGenerator.plotGeneratedVector(dialog.get_message_area())
            dialog.get_widget_for_response(Gtk.ResponseType.YES).grab_focus()
            response = dialog.run()
            dialog.destroy()

            if response == Gtk.ResponseType.YES:
                configFileAsString = coggingTorqueCalibrationGenerator.writeVectorToConfigFileString(
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
                    "Please past in the new compensation vector manually"
                )
                box = dialog.get_message_area()
                vecEntry = Gtk.Entry()
                vecEntry.set_text(coggingTorqueCalibrationGenerator.getGeneratedVectors())
                box.add(vecEntry)
                box.show_all()
                response = dialog.run()
                dialog.destroy()

    posOffset = None
    lastRefP = None

    def startCalibrationRun(nodeNr, port):
        # pylint: disable=too-many-locals, too-many-statements
        nonlocal runThread
        nonlocal posOffset
        nonlocal lastRefP

        try:
            def initFun(servoArray):
                controlSpeed, velControlSpeed, filterSpeed, inertiaMarg = contorlParameters.getValues()
                servoArray[0].setControlSpeed(controlSpeed, velControlSpeed, filterSpeed, inertiaMarg)
                servoArray[0].setBacklashControlSpeed(0.0, 3.0, 0.0)

            with createServoManager(nodeNr, port, dt=0.018, initFunction=initFun) as servoManager:
                t = 0.0
                doneRunning = False

                servoManager.servoArray[0].getPosition(False)
                refP = servoManager.servoArray[0].getPosition()
                moveHandler = SmoothMoveHandler(refP, 0.4)
                refV = 0.0

                calibrationLevelMultiplier = 2**calibrationLevelComboBox[1].get_active()

                maxVel = 0.2
                maxRecordVel = 0.01 / calibrationLevelMultiplier
                maxDist = 0.25

                if posOffset is None:
                    posOffset = refP
                elif lastRefP is not None and abs(refP - lastRefP) > 0.1:
                    posOffset = refP

                testState = 0
                endPos = -maxDist + posOffset
                moveHandler.set(endPos, maxVel)

                out = []

                abortCalibration = False

                if os.path.isfile('motorCoggingTorqueToLoad.txt') and t == 0.0:
                    dialog = Gtk.MessageDialog(
                            transient_for=parent,
                            flags=0,
                            message_type=Gtk.MessageType.INFO,
                            buttons=Gtk.ButtonsType.YES_NO,
                            text='Found file: "motorCoggingTorqueToLoad.txt"!\n'
                                'Should this file be loaded instead of\n'
                                'running a new calibration?',
                    )
                    response = dialog.run()
                    dialog.destroy()

                    if response == Gtk.ResponseType.YES:
                        out = np.loadtxt('motorCoggingTorqueToLoad.txt')
                        doneRunning = True

                def sendCommandHandlerFunction(dt, servoManager):
                    nonlocal testState
                    nonlocal refP
                    nonlocal refV
                    nonlocal lastRefP
                    nonlocal endPos

                    servo = servoManager.servoArray[0]

                    refP, refV = moveHandler.getNextRef(dt)

                    eps = 1.0e-8

                    if testState == 0:
                        if abs(refP - endPos) < eps:
                            endPos = maxDist + posOffset
                            moveHandler.set(endPos, maxRecordVel)
                            testState += 1

                    elif testState == 1:
                        if abs(refP - endPos) < eps:
                            endPos = -maxDist + posOffset
                            moveHandler.set(endPos, maxRecordVel)
                            testState += 1

                    elif testState == 2:
                        if abs(refP - endPos) < eps:
                            endPos = 0.0 + posOffset
                            moveHandler.set(endPos, maxVel)
                            testState += 1

                    elif testState == 3:
                        if abs(refP - endPos) < eps:
                            testState += 1

                    else:
                        pass

                    servo.setReference(refP, refV, 0)

                    lastRefP = refP

                def readResultHandlerFunction(dt, servoManager):
                    nonlocal t
                    nonlocal doneRunning
                    nonlocal abortCalibration

                    t += dt

                    with threadMutex:
                        if runThread is False:
                            abortCalibration = True

                    if abortCalibration or parent.isClosed or testState == 4:
                        servoManager.removeHandlerFunctions()
                        doneRunning = True
                        return

                    servo = servoManager.servoArray[0]

                    p = servo.getPosition(True)
                    v = servo.getVelocity()
                    u = servo.getPwmControlSignal()
                    error = servo.getControlError(True)
                    motorError = servo.getControlError(False)
                    optData = servo.getOpticalEncoderChannelData()

                    if testState in (1, 2) and abs(refV) >= maxRecordVel * 0.4:
                        out.append([time.time(), p, v,
                                error,
                                motorError,
                                u,
                                optData.minCostIndex])

                    GLib.idle_add(updateProgress, t /
                            ((maxDist * 2 * 2 / maxRecordVel + maxDist * 2 / maxVel) * (math.pi / 2)))

                servoManager.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction)

                while not doneRunning:
                    if not servoManager.isAlive():
                        runThread = False
                        break
                    time.sleep(0.1)

                servoManager.shutdown()

                if not abortCalibration:
                    data = np.array(out)
                    np.savetxt('lastMotorCoggingTorque.txt', data)
                    GLib.idle_add(handleResults, data)

        except Exception as e:
            GuiFunctions.exceptionMessage(parent, e)
        finally:
            GLib.idle_add(resetGuiAfterCalibration)

    testThread = None

    def onStartCalibration(widget):
        nonlocal testThread
        nonlocal runThread

        if widget.get_label() == 'Start':
            recordingProgressBar[1].set_fraction(0.0)
            calibrationBox.pack_start(recordingProgressBar[0], False, False, 0)

            widget.set_label('Abort')

            controlSpeedScale[1].set_sensitive(False)
            advancedParametersButton[1].set_sensitive(False)
            resetCalibrationButton[1].set_sensitive(False)

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

    return calibrationBox
