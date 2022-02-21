from ServoProjectModules.CalibrationAnalyzers.Helper import *

class CoggingTorqueCalibrationGenerator(object):
    def __init__(self, positions, forces):
        self.positions = np.array(positions)
        self.forces = np.array(forces)

        samplesList = []
        for i in range (0, 512):
            samplesList.append([])

        for d in zip(self.positions, self.forces):
            for i in range(-12, 13):
                i = (int(d[0] / 4) + i) % len(samplesList)
                samplesList[i].append(d[1])

        self.cogging = np.array([sum(samples) / len(samples) for samples in samplesList])

    _vecPattern = re.compile(r'(?P<beg>.*getPosDepForceCompVec\(\)\s*\{(.*\n)*?\s*(constexpr)?\s+(static)?\s+std\s*::\s*array\s*<\s*int16_t\s*,\s*512\s*>\s+vec\s*=\s*)\{\s*(?P<vec>[^\}]*)\s*\};')

    def checkForPreviousCalibration(configFileAsString, configClassName):
        configClassString = getConfigClassString(configFileAsString, configClassName)

        temp = CoggingTorqueCalibrationGenerator._vecPattern.search(configClassString)
        if temp != None:
            if temp.group('vec') != '0':
                return True

        return False

    def resetPreviousCalibration(configFileAsString, configClassName):
        configClassString = getConfigClassString(configFileAsString, configClassName)
        configClassString = re.sub(CoggingTorqueCalibrationGenerator._vecPattern, r'\g<beg>{0};', configClassString)
        configFileAsString = setConfigClassString(configFileAsString, configClassName, configClassString)

        return configFileAsString

    def plotGeneratedVector(self, box):
        fig = Figure(figsize=(5, 4), dpi=100)
        ax = fig.add_subplot()

        ax.plot(self.positions / 4, self.forces, 'g+')
        ax.plot(self.cogging, 'b')
        plt.show()

        canvas = FigureCanvas(fig)
        canvas.set_size_request(600, 400)
        box.add(canvas)

        box.show_all()

    def writeVectorToConfigFileString(self, configFileAsString, configClassName):
        configClassString = getConfigClassString(configFileAsString, configClassName)
        
        temp = CoggingTorqueCalibrationGenerator._vecPattern.search(configClassString)
        if temp != None:
            configClassString = re.sub(CoggingTorqueCalibrationGenerator._vecPattern, r'\g<beg>' + intArrayToString(self.cogging), configClassString)

            configFileAsString = setConfigClassString(configFileAsString, configClassName, configClassString)
            return configFileAsString

        return ''

    def getGeneratedVector(self):
        out = ''
        out += 'constexpr static std::array<int16_t, 512> vec = ' + intArrayToString(self.cogging)

        return out

def createGuiBox(parent, nodeNr, getPortFun, configFilePath, configClassName):
    calibrationBox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
    calibrationBox.set_margin_start(40)
    calibrationBox.set_margin_bottom(100)

    controlSpeedScale = GuiFunctions.creatHScale(14, 0, 100, 1, getLowLev=True)
    controlSpeedScale = GuiFunctions.addTopLabelTo('<b>Control speed</b>', controlSpeedScale[0]), controlSpeedScale[1]
    calibrationBox.pack_start(controlSpeedScale[0], False, False, 0)

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
        calibrationBox.remove(recordingProgressBar[0])

    runThread = False

    def handleResults(data):
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
            time = data[:, 0] - data[0, 0]
            fig = plt.figure(1)
            fig.suptitle('Position')
            plt.plot(time, data[:, 1], 'g')

            fig = plt.figure(2)
            fig.suptitle('Velocity')
            plt.plot(time, data[:, 2])

            fig = plt.figure(3)
            fig.suptitle('Error at output')
            plt.plot(time, data[:, 3])

            fig = plt.figure(4)
            fig.suptitle('Error at motor')
            plt.plot(time, data[:, 4])

            fig = plt.figure(5)
            fig.suptitle('Control signal')
            plt.plot(time, data[:, 5])

            fig = plt.figure(6)
            fig.suptitle('Control signal over motor position')
            plt.plot(np.array(data[:, 6]), data[:, 5], 'g+')

            plt.show()

        with open(configFilePath, "r") as configFile:
            configFileAsString = configFile.read()

            coggingTorqueCalibrationGenerator = CoggingTorqueCalibrationGenerator(data[:, 6], data[:, 5])

            dialog = Gtk.MessageDialog(
                    transient_for=parent,
                    flags=0,
                    message_type=Gtk.MessageType.INFO,
                    buttons=Gtk.ButtonsType.YES_NO,
                    text='Cogging torque calibration done!',
            )
            dialog.format_secondary_text(
                "Should the configuration be updated with the blue compensation vector?"
            )
            coggingTorqueCalibrationGenerator.plotGeneratedVector(dialog.get_message_area())
            dialog.get_widget_for_response(Gtk.ResponseType.YES).grab_focus()
            response = dialog.run()
            dialog.destroy()

            if response == Gtk.ResponseType.YES:
                configFileAsString = coggingTorqueCalibrationGenerator.writeVectorToConfigFileString(configFileAsString, configClassName)

                if configFileAsString != '':
                    with open(configFilePath, "w") as configFile:
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
                vecEntry.set_text(coggingTorqueCalibrationGenerator.getGeneratedVector())
                box.add(vecEntry)
                box.show_all()
                response = dialog.run()
                dialog.destroy()

    posOffset = None
    lastRefP = None

    def startCalibrationRun(nodeNr, port):
        nonlocal runThread
        nonlocal posOffset
        nonlocal lastRefP

        controlSpeed = int(controlSpeedScale[1].get_value())

        def initFun(servoManager):
            servoManager.servoArray[0].setControlSpeed(controlSpeed)
            servoManager.servoArray[0].setBacklashControlSpeed(0.0, 3.0, 0.0)

        with createServoManager(nodeNr, port, dt=0.018, initFunction=initFun) as servoManager:
            t = 0.0
            doneRunning = False

            servoManager.servoArray[0].getPosition(False)
            refP = servoManager.servoArray[0].getPosition()
            moveHandler = SmoothMoveHandler(refP, 0.4)
            refV = 0.0

            maxVel = 0.2
            maxRecordVel = 0.1
            maxDist = 0.25

            if posOffset == None:
                posOffset = refP
            elif lastRefP != None and abs(refP - lastRefP) > 0.1:
                posOffset = refP

            testState = 0
            endPos = -maxDist + posOffset
            moveHandler.set(endPos, maxVel)

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

            out = []

            abortCalibration = False

            def readResultHandlerFunction(dt, servoManager):
                nonlocal t
                nonlocal doneRunning
                nonlocal abortCalibration

                t += dt

                with threadMutex:
                    if runThread == False:
                        abortCalibration = True

                if abortCalibration or parent.isClosed or testState == 4:
                    servoManager.removeHandlerFunctions()
                    doneRunning = True
                    return

                servo = servoManager.servoArray[0]

                p = servo.getPosition(True)
                v = servo.getVelocity()
                u = servo.getControlSignal()
                error = servo.getControlError(True)
                motorError = servo.getControlError(False)
                optData = servo.getOpticalEncoderChannelData()

                if (testState == 1 or testState == 2) and abs(refV) >= maxRecordVel * 0.4:
                        out.append([time.time(), p, v,
                                error,
                                motorError,
                                u,
                                optData.minCostIndex])

                GLib.idle_add(updateProgress, t / ((0.5 * 2 / maxRecordVel + 0.25 * 2 / maxVel) * (math.pi / 2)))

            servoManager.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction);

            while not doneRunning:
                if not servoManager.isAlive():
                    runThread = False
                    break
                time.sleep(0.1)

            servoManager.shutdown()

            if not abortCalibration:
                data = np.array(out)
                GLib.idle_add(handleResults, data)

        GLib.idle_add(resetGuiAfterCalibration)

    testThread = None

    def onStartCalibration(widget):
        nonlocal testThread
        nonlocal runThread

        if widget.get_label() == 'Start':
            with open(configFilePath, "r") as configFile:
                configFileAsString = configFile.read()

                if CoggingTorqueCalibrationGenerator.checkForPreviousCalibration(configFileAsString, configClassName):
                    dialog = Gtk.MessageDialog(
                            transient_for=parent,
                            flags=0,
                            message_type=Gtk.MessageType.ERROR,
                            buttons=Gtk.ButtonsType.YES_NO,
                            text='Motor cogging calibration already done for this configuration!',
                    )
                    dialog.format_secondary_text(
                        "Motor cogging calibration only works on configurations without previous calibration.\n\nShould the calibration be reset?"
                    )
                    response = dialog.run()
                    dialog.destroy()

                    if response == Gtk.ResponseType.NO:
                        return

                    configFileAsString = CoggingTorqueCalibrationGenerator.resetPreviousCalibration(configFileAsString, configClassName)
                    with open(configFilePath, "w") as configFile:
                        configFile.write(configFileAsString)
                        GuiFunctions.transferToTargetMessage(parent)

                    return

            recordingProgressBar[1].set_fraction(0.0)
            calibrationBox.pack_start(recordingProgressBar[0], False, False, 0)

            widget.set_label('Abort')

            controlSpeedScale[1].set_sensitive(False)

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
