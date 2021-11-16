from ServoProjectModules.CalibrationAnalyzers.Helper import *

class PwmNonlinearityIdentifier(object):
    def __init__(self, data):
        self.data = data

        self.lookUpStepSize = 4

        self.timeList = []
        pwmList = []
        self.ampList = []
        ampSum = 0
        ampSumNr = 0
        startOfPwmValue = 0
        lastPwm = None
        for d in zip(self.data[:, 0], self.data[:, 1], self.data[:, 2]):
            if not lastPwm == d[2]:
                if not lastPwm == None:
                    self.timeList.append(d[0])
                    pwmList.append(lastPwm)
                    self.ampList.append(math.sqrt(ampSum / ampSumNr))

                lastPwm = d[2]
                startOfPwmValue = d[0]
                ampSum = 0
                ampSumNr = 0

            if d[0] - startOfPwmValue > 2.0:
                ampSum += d[1]**2
                ampSumNr += 1

        self.timeList.append(self.data[-1, 0])
        pwmList.append(lastPwm)
        self.ampList.append(math.sqrt(ampSum / ampSumNr))

        i = len(pwmList) - 1
        
        t = (1023 - pwmList[i - 1]) / (pwmList[i] - pwmList[i - 1])

        pwmList.append((pwmList[i] - pwmList[i - 1]) * t + pwmList[i - 1])
        self.timeList.append((self.timeList[i] - self.timeList[i - 1]) * t + self.timeList[i - 1])
        self.ampList.append(((self.ampList[i] * pwmList[i] - self.ampList[i - 1] * pwmList[i - 1]) * t 
            + self.ampList[i - 1] * pwmList[i - 1]) / 1023)

        lastZero = 0
        self.compList = [0]
        self.compListPwm = [0]
        for i, d in enumerate(zip(pwmList, self.ampList)):
            comp = d[0] * d[1] / self.ampList[-1]
            self.compList.append(comp)
            self.compListPwm.append(d[0])
            if comp < 20:
                lastZero = i + 1

        firstNoneZeroVal = self.compList[lastZero + 1]
        firstNoneZeroValPwm = self.compListPwm[lastZero + 1]

        for i in range(0, lastZero + 1):
            pwm = self.compListPwm[i]
            newValue = pwm**2 / firstNoneZeroValPwm**2 * firstNoneZeroVal
            self.compList[i] = newValue

#        currList = [0, 0.0026, 0.005, 0.0082, 0.0121, 0.0159, 0.0202, 0.025, 0.0305, 0.0361, 0.0424, 0.0488, 0.0566, 0.078, 0.107, 0.1533, 0.18, 0.24, 0.33, 0.42, 0.57, 0.72, 0.99, 1.35]
#        pwmList = [0, 50, 100, 150, 200, 250, 300, 350, 400, 450, 500, 550, 600, 650, 680, 720, 735, 760, 793, 825, 869, 908, 960, 1023]
#        self.compList = np.array(currList) / currList[-1] * 1023
#        self.compListPwm = pwmList
#
        self.pwmNonlinearityCompLookUp = []

        for pwm in range(0, 1024, self.lookUpStepSize):
            index = -1
            for d in self.compList[0:-1]:
                if d < pwm:
                    index += 1
                else:
                    break

            t = (pwm - self.compList[index]) / (self.compList[index + 1] - self.compList[index])
            self.pwmNonlinearityCompLookUp.append(self.compListPwm[index] + (self.compListPwm[index + 1] - self.compListPwm[index]) * t)

    def plotGeneratedVector(self, box):
        fig = Figure(figsize=(5, 4), dpi=100)
        ax = fig.add_subplot()

        ax.plot(range(0, 1024, self.lookUpStepSize), range(0, 1024, self.lookUpStepSize), 'k-')
        ax.plot(range(0, 1024, self.lookUpStepSize), self.pwmNonlinearityCompLookUp, 'g+-')
        ax.plot(self.compList, self.compListPwm, 'c.')

        canvas = FigureCanvas(fig)
        canvas.set_size_request(600, 400)
        box.add(canvas)

        box.show_all()


    def writeLinearizationFunctionToConfigClassString(self, configClassString):
        linearizeVecPattern = re.compile(r'((\s*).*createCurrentController\(\)\s*\{(.*\n)*?)(\s*)auto\s+pwmHighFrqCompFun\s+=\s+\[\]\(uint16_t\s+in\)(.*\n)*?\4\};((.*\n)*?\2\})')
        
        temp = linearizeVecPattern.search(configClassString)
        if temp != None:
            lookUpSize = len(self.pwmNonlinearityCompLookUp)
            out = r'\1'
            out += self.getLinearizationFunction(r'\4')
            out += r'\6'
            configClassString = re.sub(linearizeVecPattern, out, configClassString)

            return configClassString

        return ''

    def getLinearizationFunction(self, indent = ''):
        lookUpSize = len(self.pwmNonlinearityCompLookUp)
        out = ''
        out += indent + 'auto pwmHighFrqCompFun = [](uint16_t in)\n'
        out += indent + '{\n'
        out += indent + '    constexpr static std::array<uint16_t, ' + str(lookUpSize) + '> linearizeVec = ' + intArrayToString(self.pwmNonlinearityCompLookUp) + '\n'
        out += '\n'
        out += indent + '    float t = in * (' + str(lookUpSize - 1.0) + 'f / 1023.0f);\n'
        out += indent + '    size_t index = std::min(static_cast<size_t>(t), (size_t)' + str(lookUpSize - 2) + ');\n'
        out += indent + '    t -= index;\n'
        out += indent + '    const uint16_t& a = linearizeVec[index];\n'
        out += indent + '    const uint16_t& b = linearizeVec[index + 1];\n'
        out += '\n'
        out += indent + '    return static_cast<uint16_t>((b - a) * t + a);\n'
        out += indent + '};'

        return out

def createGuiBox(parent, nodeNr, port, configFilePath, configClassName):
    calibrationBox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
    calibrationBox.set_margin_start(40)

    maxOscillationFrq = 10 
    maxFrqScale = GuiFunctions.creatHScale(maxOscillationFrq, 1, 100, 1, getLowLev=True)
    maxFrqScale = GuiFunctions.addTopLabelTo('<b>Max oscillation frequency</b>', maxFrqScale[0]), maxFrqScale[1]
    calibrationBox.pack_start(maxFrqScale[0], False, False, 0)

    minPwmValue = 0
    midPwmValue = 500
    maxPwmValue = 605
    minPwmScale = GuiFunctions.creatHScale(minPwmValue, 0, 1023, 10, getLowLev=True)
    minPwmScale = GuiFunctions.addTopLabelTo('<b>Pwm value to start calibration at</b>\n This should be just under the pwm value that makes the motor start moving', minPwmScale[0]), minPwmScale[1]
    calibrationBox.pack_start(minPwmScale[0], False, False, 0)

    midPwmScale = GuiFunctions.creatHScale(midPwmValue, 0, 1023, 10, getLowLev=True)
    midPwmScale = GuiFunctions.addTopLabelTo('<b>Pwm value for switching to sparse sample points</b>\n Lower value limits motor heat up at the cost of calibration resolution', midPwmScale[0]), midPwmScale[1]
    calibrationBox.pack_start(midPwmScale[0], False, False, 0)

    maxPwmScale = GuiFunctions.creatHScale(maxPwmValue, 1, 1023, 10, getLowLev=True)
    maxPwmScale = GuiFunctions.addTopLabelTo('<b>Max motor pwm value</b>', maxPwmScale[0]), maxPwmScale[1]
    calibrationBox.pack_start(maxPwmScale[0], False, False, 0)

    testButton = GuiFunctions.createButton('Test pwm value', getLowLev=True)
    startButton = GuiFunctions.createButton('Start calibration', getLowLev=True)

    recordingProgressBar = GuiFunctions.creatProgressBar(label='Recording', getLowLev=True)

    testPwmValue = midPwmValue

    threadMutex = threading.Lock()
    def maxFrqValueChanged(widget):
        nonlocal maxOscillationFrq
        nonlocal threadMutex

        with threadMutex:
            maxOscillationFrq = widget.get_value()

    def minPwmValueChanged(widget):
        nonlocal maxPwmScale
        nonlocal midPwmValue
        nonlocal minPwmValue
        nonlocal testPwmValue
        nonlocal threadMutex

        with threadMutex:
            minPwmValue = widget.get_value()
            testPwmValue = minPwmValue

        if minPwmValue > midPwmScale[1].get_value():
            midPwmScale[1].set_value(minPwmValue)

        if minPwmValue > maxPwmScale[1].get_value():
            maxPwmScale[1].set_value(minPwmValue)

    def midPwmValueChanged(widget):
        nonlocal maxPwmScale
        nonlocal midPwmValue
        nonlocal minPwmValue
        nonlocal testPwmValue
        nonlocal threadMutex

        with threadMutex:
            midPwmValue = widget.get_value()
            testPwmValue = midPwmValue

        if midPwmValue < minPwmScale[1].get_value():
            minPwmScale[1].set_value(midPwmValue)

        if midPwmValue > maxPwmScale[1].get_value():
            maxPwmScale[1].set_value(midPwmValue)

    def maxPwmValueChanged(widget):
        nonlocal midPwmScale
        nonlocal maxPwmValue
        nonlocal testPwmValue
        nonlocal threadMutex

        with threadMutex:
            maxPwmValue = widget.get_value()
            testPwmValue = maxPwmValue

        if maxPwmValue < minPwmScale[1].get_value():
            minPwmScale[1].set_value(maxPwmValue)

        if maxPwmValue < midPwmScale[1].get_value():
            midPwmScale[1].set_value(maxPwmValue)

    maxFrqScale[1].connect('value-changed', maxFrqValueChanged)
    minPwmScale[1].connect('value-changed', minPwmValueChanged)
    midPwmScale[1].connect('value-changed', midPwmValueChanged)
    maxPwmScale[1].connect('value-changed', maxPwmValueChanged)

    def resetGuiAfterCalibration():
        nonlocal startButton
        nonlocal testButton
        nonlocal calibrationBox
        nonlocal recordingProgressBar
        nonlocal maxFrqScale
        nonlocal minPwmScale
        nonlocal midPwmScale
        nonlocal maxPwmScale

        startButton[1].set_label('Start calibration')
        startButton[1].set_sensitive(True)
        testButton[1].set_label('Test pwm value')
        testButton[1].set_sensitive(True)
        calibrationBox.remove(recordingProgressBar[0])
        maxFrqScale[1].set_sensitive(True)
        minPwmScale[1].set_sensitive(True)
        midPwmScale[1].set_sensitive(True)
        maxPwmScale[1].set_sensitive(True)

    resetGuiAfterCalibration()

    runThread = False
    def testPwmRun(nodeNr, port):
        nonlocal runThread
        nonlocal threadMutex

        try:
            robot = createRobot(nodeNr, port)

            t = 0.0
            doneRunning = False
            pwm = 0

            def sendCommandHandlerFunction(dt, robot):
                nonlocal nodeNr
                nonlocal t
                nonlocal maxOscillationFrq
                nonlocal maxPwmValue
                nonlocal testPwmValue
                nonlocal threadMutex
                nonlocal pwm

                servo = robot.dcServoArray[nodeNr - 1]

                frq = 0
                pwmAmp = 0
                with threadMutex:
                    pwmAmp = testPwmValue
                    frq = pwmAmp / maxPwmValue * maxOscillationFrq

                pwm = pwmAmp * math.sin(frq * 2 * pi * t)
                servo.setOpenLoopControlSignal(pwm, True)

            out = []

            lastPosition = None

            def readResultHandlerFunction(dt, robot):
                nonlocal nodeNr
                nonlocal t
                nonlocal runThread
                nonlocal doneRunning
                nonlocal pwm
                nonlocal lastPosition

                t += dt
                servo = robot.dcServoArray[nodeNr - 1]

                position = servo.getPosition(False)

                if lastPosition == None:
                    lastPosition = position

                out.append([t, (position - lastPosition) / dt, pwm])

                lastPosition = position

                stop = False
                with threadMutex:
                    if runThread == False:
                        stop = True

                if stop or parent.isClosed:
                    robot.removeHandlerFunctions()
                    doneRunning = True

            robot.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction);

            while not doneRunning:
                time.sleep(0.1)

            robot.shutdown()

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
                    plt.figure(1)
                    plt.plot(data[:, 0], data[:, 1], 'r')
                    plt.figure(2)
                    plt.plot(data[:, 0], data[:, 2], 'b')
                    plt.show()

            GLib.idle_add(plotData, data)
        except Exception as e:
            print(format(e))

        GLib.idle_add(resetGuiAfterCalibration)

    testThread = None

    def onTestPwm(widget):
        nonlocal calibrationBox
        nonlocal startButton

        nonlocal testThread
        nonlocal runThread
        nonlocal threadMutex

        if widget.get_label() == 'Test pwm value':
            startButton[1].set_sensitive(False)
            widget.set_label('Stop pwm test')
            with threadMutex:
                runThread = True
            testThread = threading.Thread(target=testPwmRun, args=(nodeNr,port,))
            testThread.start()
        else:
            with threadMutex:
                runThread = False
            testThread.join()

    def updateRecordingProgressBar(fraction):
        nonlocal recordingProgressBar

        recordingProgressBar[1].set_fraction(fraction)

    def handleResults(data):
        pwmNonlinearityIdentifier = PwmNonlinearityIdentifier(data)

        dialog = Gtk.MessageDialog(
                transient_for=parent,
                flags=0,
                message_type=Gtk.MessageType.INFO,
                buttons=Gtk.ButtonsType.YES_NO,
                text='Pwm linearity calibration done!',
        )
        dialog.format_secondary_text(
            "Should the configuration be updated with the new data?"
        )
        pwmNonlinearityIdentifier.plotGeneratedVector(dialog.get_message_area())
        response = dialog.run()
        dialog.destroy()

        if response == Gtk.ResponseType.YES:
            with open(configFilePath, "r") as configFile:
                configFileAsString = configFile.read()

                classPattern =re.compile(r'(\s*class\s+' + configClassName + r'\s+(.*\n)*?(.*\n)*?\})') 
                temp = classPattern.search(configFileAsString)
                if temp != None:
                    classString = temp.group(0)

                    classString = pwmNonlinearityIdentifier.writeLinearizationFunctionToConfigClassString(classString)
                    if classString != '':
                        configFileAsString = re.sub(classPattern, classString, configFileAsString)
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
                "Please past in the linearization function manually"
            )
            box = dialog.get_message_area()
            funEntry = Gtk.Entry()
            funEntry.set_text(pwmNonlinearityIdentifier.getLinearizationFunction())
            box.add(funEntry)
            box.show_all()
            response = dialog.run()
            dialog.destroy()

    def startCalibrationRun(nodeNr, port):
        nonlocal runThread
        nonlocal threadMutex
        nonlocal maxOscillationFrq
        nonlocal maxPwmValue

        try:
            robot = createRobot(nodeNr, port)

            with threadMutex:
                highResStep = min(25, int(midPwmValue - minPwmValue / 10.0))
                temp = [v for v in range(int(minPwmValue), int(midPwmValue), highResStep)]
                temp = np.array(temp)
                temp += highResStep
                temp = temp / temp[-1] * midPwmValue

                temp2 = [v for v in range(0, int(maxPwmValue - midPwmValue), 100)]
                temp2 = np.array(temp2)
                temp2 += 100
                temp2 = temp2 / temp2[-1] * (maxPwmValue - midPwmValue)
                temp2 += midPwmValue
                pwmSampleValues = np.append(temp, temp2)

                temp = []
                for v in pwmSampleValues:
                    frq = v / maxPwmValue * maxOscillationFrq

                    sampleTime = max(4.0, 2.0 + 10 / frq)

                    i = 0
                    while i < sampleTime:
                        i += 1
                        temp.append(v)

                pwmSampleValues = np.array(temp)

            t = 0.0
            runTime = 6.0
            doneRunning = False

            def sendCommandHandlerFunction(dt, robot):
                nonlocal nodeNr
                nonlocal t
                nonlocal threadMutex
                nonlocal maxOscillationFrq
                nonlocal maxPwmValue

                t += dt

                servo = robot.dcServoArray[nodeNr - 1]

                pwmAmp = 0.0
                if int(t) < len(pwmSampleValues):
                    pwmAmp = pwmSampleValues[int(t)]
                else:
                    return

                frq = 0.0
                with threadMutex:
                    frq = pwmAmp / maxPwmValue * maxOscillationFrq

                GLib.idle_add(updateRecordingProgressBar, t / len(pwmSampleValues))

                pwm = pwmAmp * math.sin(frq * 2 * pi * t)
                servo.setOpenLoopControlSignal(pwm, True)

            out = []

            lastPosition = None

            def readResultHandlerFunction(dt, robot):
                nonlocal nodeNr
                nonlocal t
                nonlocal runThread
                nonlocal doneRunning
                nonlocal pwmSampleValues
                nonlocal lastPosition

                stop = int(t) >= len(pwmSampleValues)
                with threadMutex:
                    if runThread == False:
                        stop = True

                if stop or parent.isClosed:
                    robot.removeHandlerFunctions()
                    doneRunning = True
                    return

                servo = robot.dcServoArray[nodeNr - 1]

                position = servo.getPosition(False)

                if lastPosition == None:
                    lastPosition = position

                out.append([t, (position - lastPosition) / dt, pwmSampleValues[int(t)]])

                lastPosition = position

            robot.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction);

            while not doneRunning:
                time.sleep(0.1)

            robot.shutdown()

            if runThread == True:
                data = np.array(out)
                GLib.idle_add(handleResults, data)

        except Exception as e:
            print(format(e))

        GLib.idle_add(resetGuiAfterCalibration)

    def onStartCalibration(widget):
        nonlocal testThread
        nonlocal threadMutex
        nonlocal runThread

        nonlocal calibrationBox
        nonlocal maxFrqScale
        nonlocal minPwmScale
        nonlocal midPwmScale
        nonlocal maxPwmScale
        nonlocal recordingProgressBar
        nonlocal testButton

        if widget.get_label() == 'Start calibration':
            with open(configFilePath, "r") as configFile:
                configFileAsString = configFile.read()

                classPattern =re.compile(r'(\s*class\s+' + configClassName + r'\s+(.*\n)*?(.*\n)*?\})') 
                classString = classPattern.search(configFileAsString).group(0)

                linearizeVecPattern = re.compile(r'(\n([ \t]*).*createCurrentController\(\)\s*\{(.*\n)*?(\s*)auto\s+pwmHighFrqCompFun\s+=\s+\[\]\(uint16_t\s+in\)\4\{\n)([ \t]*)((.*\n)*?.*)(\4\};(.*\n)*?\2\})')

                if linearizeVecPattern.search(classString).group(6) != 'return in;':
                    dialog = Gtk.MessageDialog(
                            transient_for=parent,
                            flags=0,
                            message_type=Gtk.MessageType.ERROR,
                            buttons=Gtk.ButtonsType.YES_NO,
                            text='Pwm calibration already done for this configuration!',
                    )
                    dialog.format_secondary_text(
                        "Pwm linarization only works on configurations without previous calibration.\n\nShould the calibration be reset?"
                    )
                    response = dialog.run()
                    dialog.destroy()

                    if response == Gtk.ResponseType.NO:
                        return

                    classString = re.sub(linearizeVecPattern, r'\1\5return in;\8', classString)
                    configFileAsString = re.sub(classPattern, classString, configFileAsString)
                    with open(configFilePath, "w") as configFile:
                        configFile.write(configFileAsString)
                        GuiFunctions.transferToTargetMessage(parent)

                    return

            widget.set_label('Abort calibration')

            recordingProgressBar[1].set_fraction(0.0)
            testButton[1].set_sensitive(False)
            calibrationBox.pack_start(recordingProgressBar[0], False, False, 0)
            maxFrqScale[1].set_sensitive(False)
            minPwmScale[1].set_sensitive(False)
            midPwmScale[1].set_sensitive(False)
            maxPwmScale[1].set_sensitive(False)

            calibrationBox.show_all()

            with threadMutex:
                runThread = True
            testThread = threading.Thread(target=startCalibrationRun, args=(nodeNr, port,))
            testThread.start()                                    
        else:
            with threadMutex:
                runThread = False
            testThread.join()

    testButton[1].connect('clicked', onTestPwm)
    startButton[1].connect('clicked', onStartCalibration)
    calibrationBox.pack_start(testButton[0], False, False, 0)
    calibrationBox.pack_start(startButton[0], False, False, 0)
    calibrationBox.show_all()
    return calibrationBox
