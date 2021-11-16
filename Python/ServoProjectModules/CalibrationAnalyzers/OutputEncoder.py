from ServoProjectModules.CalibrationAnalyzers.Helper import *

class OutputEncoderCalibrationGenerator(object):
    def __init__(self, data, wrapAround, unitsPerRev):
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

        for d in self.data:
            if d[1] != maxPos and d[1] != minPos:
                pos = int(round((d[1] % 4096) / 8))
                if pos == posListSize:
                    pos = posListSize - 1
                posList[pos].append(d[2])

        backlashSizeSum = 0
        backlashSizeNr = 0
        for d in posList:
            if len(d) >= 2:
                backlashSizeSum += max(d) - min(d)
                backlashSizeNr += 1

        meanBacklashSize = backlashSizeSum / backlashSizeNr

        self.minList = []
        self.maxList = []
        self.meanList = []
        for d in posList:
            temp = d
            if len(temp) == 0:
                self.minList.append(None)
                self.maxList.append(None)
                self.meanList.append(None)
            else:
                maxV = max(temp)
                minV = min(temp)
                meanV = (maxV + minV) / 2
                minSum = 0
                minNr = 0
                maxSum = 0
                maxNr = 0
                for d in temp:
                    if d < meanV - meanBacklashSize / 2 * 0.0:
                        minSum += d
                        minNr += 1
                    elif d > meanV + meanBacklashSize / 2 * 0.0:
                        maxSum += d
                        maxNr += 1

                if minNr > 1 and maxNr > 1:
                    self.minList.append(minSum / minNr)
                    self.maxList.append(maxSum / maxNr)
                    self.meanList.append((self.minList[-1] + self.maxList[-1]) / 2)
                else:
                    self.minList.append(None)
                    self.maxList.append(None)
                    self.meanList.append(None)

        i = 0
        if wrapAround:
            while self.meanList[i - 1] == None:
                i -= 1

        noneSegList = []
        end = i + len(self.meanList)
        while True:
            noneSeg = []
            while self.meanList[i] != None:
                i += 1
                if i == end:
                    break
            if i == end:
                break
            while self.meanList[i] == None:
                noneSeg.append(i)
                i += 1
                if i == end:
                    break
            noneSegList.append(noneSeg)
            if i == end:
                break

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

    def checkForInvertedEncoder(data):
        minPos = min(data[:, 1])
        maxPos = max(data[:, 1])
        minDiff = min(data[:, 2])
        maxDiff = max(data[:, 2])

        meanCompSlope = abs(maxDiff - minDiff) / abs(maxPos - minPos)

        return meanCompSlope > 1.5

    def isInverted(self):
        return OutputEncoderCalibrationGenerator.checkForInvertedEncoder(self.data)

    def plotGeneratedVector(self, box):
        fig = Figure(figsize=(5, 4), dpi=100)
        ax = fig.add_subplot()

        ax.plot(self.data[:, 1] % 4096, self.data[:, 2], 'b,')
        #ax.plot(self.minList, 'b-+')
        #ax.plot(self.maxList, 'r-+')
        ax.plot(range(0, 4096 + 8, 8), self.meanList, 'g-+')

        canvas = FigureCanvas(fig)
        canvas.set_size_request(600, 400)
        box.add(canvas)

        box.show_all()

    def writeVectorToConfigClassString(self, configClassString):
        compVecPattern = re.compile(r'(.*createOutputEncoderHandler\(\)\s*\{(.*\n)*?\s*std\s*::\s*array\s*<\s*int16_t\s*,\s*513\s*>\s*compVec\s*=\s*)\{\s*[^\}]*\s*\};')
        
        temp = compVecPattern.search(configClassString)
        if temp != None:
            configClassString = re.sub(compVecPattern, r'\1' + intArrayToString(self.meanList), configClassString)

            return configClassString

        return ''

    def getGeneratedVector(self):
        out = ''
        out += 'std::array<int16_t, 513 > compVec = ' + intArrayToString(self.meanList)

        return out

    def invertOutputEncoder(self, configClassString):
        unitPerRevPattern = re.compile(r'(.*createOutputEncoderHandler\(\)\s*\{(.*\n)*?\s*return\s+std::make_unique\s*<\s*\w*\s*>\s*\([^,]*,\s*)([^,]*)(,\s*compVec\s*\)\s*;)')

        temp = unitPerRevPattern.search(configClassString)

        if temp != None:
            configClassString = re.sub(unitPerRevPattern, r'\1-(\3)\4', configClassString)
        
            return configClassString

        return ''

def createGuiBox(parent, nodeNr, port, configFilePath, configClassName):
    calibrationBox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
    calibrationBox.set_margin_start(40)

    controlSpeedScale = GuiFunctions.creatHScale(14, 0, 100, 1, getLowLev=True)
    controlSpeedScale = GuiFunctions.addTopLabelTo('<b>Control speed</b>', controlSpeedScale[0]), controlSpeedScale[1]
    calibrationBox.pack_start(controlSpeedScale[0], False, False, 0)

    startButton = GuiFunctions.createButton('Start calibration', getLowLev=True)

    recordingProgressBar = GuiFunctions.creatProgressBar(label='Recording', getLowLev=True)
    directionLabel = GuiFunctions.createLabel('')
    directionLabel.set_margin_start(50)

    threadMutex = threading.Lock()
    testThread = None

    def resetGuiAfterCalibration():
        nonlocal startButton
        nonlocal calibrationBox
        nonlocal recordingProgressBar
        nonlocal directionLabel

        controlSpeedScale[1].set_sensitive(True)
        startButton[1].set_label('Start calibration')
        startButton[1].set_sensitive(True)
        calibrationBox.remove(recordingProgressBar[0])
        calibrationBox.remove(directionLabel)

    runThread = False

    def updateRecordingProgressBar(fraction, pos):
        nonlocal recordingProgressBar
        nonlocal directionLabel
        stepsStr = []
        stepsStr.append('1) Move the servo-output-shaft over its range of motion.\n')
        stepsStr.append('2) Leave it someware in the middle.\n')
        stepsStr.append('3) Apply constant torque in CW direction (60 sec).\n')
        stepsStr.append('4) Change torque direction to CCW (60 sec).\n')
        if fraction < 0:
            if fraction <= -1:
                recordingProgressBar[1].set_text('Waiting for move to complete')

                stepsStr[0] = '<b>' + stepsStr[0] + '</b>\n'
            else:
                recordingProgressBar[1].set_text('Confirming end positions')

                stepsStr[1] = '<b>' + stepsStr[1] + '</b>\n'

            fraction = -fraction
        else:
            if fraction < 0.5:
                fraction = fraction * 2.0
                recordingProgressBar[1].set_text('Recording with CW torque')

                stepsStr[2] = '<b>' + stepsStr[2] + '</b>\n'
                
            else:
                fraction = (fraction - 0.5) * 2.0
                recordingProgressBar[1].set_text('Recording with CCW torque')

                stepsStr[3] = '<b>' + stepsStr[3] + '</b>\n'

        directionLabel.set_text("".join(stepsStr) + '\nRaw encoder position is ' + str(int(pos)))
        directionLabel.set_use_markup(True)
        recordingProgressBar[1].set_fraction(fraction)

    def handleResults(data):
        with open(configFilePath, "r") as configFile:
            configFileAsString = configFile.read()

            classPattern =re.compile(r'(\s*class\s+' + configClassName + r'\s+(.*\n)*?(.*\n)*?\})') 
            classString = classPattern.search(configFileAsString).group(0)

            wrapAroundAndUnitPerRevPattern = re.compile(r'return\s+std::make_unique\s*<\s*(\w*)\s*>\s*\(([^;]*)\s*compVec\s*\)\s*;')

            temp = wrapAroundAndUnitPerRevPattern.search(classString)
            
            magneticEncoder = temp.group(1) == 'EncoderHandler'
            unitsPerRev = 4096
            if not magneticEncoder:
                paramStr = wrapAroundAndUnitPerRevPattern.search(classString).group(2)
                i = paramStr.find(',')
                paramStr = paramStr[i + 1:]
                i = paramStr.find(',')
                paramStr = paramStr[0:i]
                paramStr = re.sub(r'f', '', paramStr)
                unitsPerRev = eval(paramStr)

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
                    "Should the output encoder be inverted in the configuration?"
                )
                response = dialog.run()
                dialog.destroy()

                if response == Gtk.ResponseType.YES:
                    classString = outputEncoderCalibrationGenerator.invertOutputEncoder(classString)

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
                        "Please invert the output encoder manually"
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
                "Should the configuration be updated with the green compensation vector?"
            )
            outputEncoderCalibrationGenerator.plotGeneratedVector(dialog.get_message_area())
            response = dialog.run()
            dialog.destroy()

            if response == Gtk.ResponseType.YES:
                classString = outputEncoderCalibrationGenerator.writeVectorToConfigClassString(classString)

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
                    "Please past in the new compensation vector manually"
                )
                box = dialog.get_message_area()
                vecEntry = Gtk.Entry()
                vecEntry.set_text(outputEncoderCalibrationGenerator.getGeneratedModel())
                box.add(vecEntry)
                box.show_all()
                response = dialog.run()
                dialog.destroy()

    def startCalibrationRun(nodeNr, port):
        nonlocal runThread
        nonlocal threadMutex
        nonlocal controlSpeedScale

        try:
            controlSpeedScale[1].set_sensitive(False)
            controlSpeed = controlSpeedScale[1].get_value()
            def initFun(robot):
                nonlocal controlSpeed
                robot.dcServoArray[0].setControlSpeed(controlSpeed, 4 * controlSpeed, 32 * controlSpeed)
                robot.dcServoArray[0].setBacklashControlSpeed(0.0, 3.0, 0.0)

            robot = createRobot(nodeNr, port, dt=0.018, initFunction=initFun)

            t = -6.2
            doneRunning = False
            refPos = 0.0
            minPos = None
            maxPos = None
            direction = 1

            def sendCommandHandlerFunction(dt, robot):
                nonlocal t
                nonlocal threadMutex
                nonlocal refPos
                nonlocal minPos
                nonlocal maxPos
                nonlocal direction

                servo = robot.dcServoArray[0]

                if t < 0:
                    servo.setOpenLoopControlSignal(0, True)
                else:
                    refVel = (maxPos - minPos) / 10.0
                    refPos += direction * refVel * dt

                    if refPos >= maxPos:
                        refPos = maxPos
                        direction = -1
                    elif refPos <= minPos:
                        refPos = minPos
                        direction = 1

                    servo.setReference(refPos, direction * refVel, 0.0)

            out = []

            encPos = None
            filteredVel = 0.0

            def readResultHandlerFunction(dt, robot):
                nonlocal t
                nonlocal runThread
                nonlocal doneRunning
                nonlocal encPos
                nonlocal refPos
                nonlocal minPos
                nonlocal maxPos
                nonlocal filteredVel
                nonlocal out

                servo = robot.dcServoArray[0]

                runTime = 120.0

                newMotorPos = servo.getPosition(False)
                newEncPos = servo.getPosition(True)
                newVel = servo.getVelocity()

                rawEncPos = (newEncPos - servo.getOffset()) / servo.getScaling()

                out.append([t,
                        rawEncPos,
                        (newEncPos - newMotorPos) / servo.getScaling()])

                if t < 0.0:
                    if encPos == None:
                        encPos = newEncPos
                    else:
                        noiseLevel = abs(abs(newEncPos - encPos) - abs(newVel * dt))
                        encPos = newEncPos
                        filteredVel = 0.95 * filteredVel + 0.05 * newVel

                        if minPos == None:
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
                    if runThread == False:
                        stop = True
                if stop or parent.isClosed:
                    robot.removeHandlerFunctions()
                    doneRunning = True
                    return

            robot.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction)

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
        nonlocal recordingProgressBar
        nonlocal directionLabel

        if widget.get_label() == 'Start calibration':
            with open(configFilePath, "r") as configFile:
                configFileAsString = configFile.read()

                classPattern =re.compile(r'(\s*class\s+' + configClassName + r'\s+(.*\n)*?(.*\n)*?\})')
                temp = classPattern.search(configFileAsString)
                if temp != None:
                    classString = temp.group(0)

                    compVecPattern = re.compile(r'(.*createOutputEncoderHandler\(\)\s*\{(.*\n)*?\s*std\s*::\s*array\s*<\s*int16_t\s*,\s*513\s*>\s*compVec\s*=\s*)\{\s*([^\}]*)\s*\};')

                    temp = compVecPattern.search(classString)
                    if temp != None:
                        if temp.group(3) != '0':
                            dialog = Gtk.MessageDialog(
                                    transient_for=parent,
                                    flags=0,
                                    message_type=Gtk.MessageType.ERROR,
                                    buttons=Gtk.ButtonsType.YES_NO,
                                    text='Output encoder calibration already done for this configuration!',
                            )
                            dialog.format_secondary_text(
                                "Output encoder calibration only works on configurations without previous calibration.\n\nShould the calibration be reset?"
                            )
                            response = dialog.run()
                            dialog.destroy()

                            if response == Gtk.ResponseType.NO:
                                return

                            classString = re.sub(compVecPattern, r'\1{0};', classString)
                            configFileAsString = re.sub(classPattern, classString, configFileAsString)
                            with open(configFilePath, "w") as configFile:
                                configFile.write(configFileAsString)
                                GuiFunctions.transferToTargetMessage(parent)

                            return

            widget.set_label('Abort calibration')

            recordingProgressBar[1].set_fraction(1.0)
            calibrationBox.pack_start(recordingProgressBar[0], False, False, 0)
            directionLabel.set_text('')
            calibrationBox.pack_start(directionLabel, False, False, 0)

            calibrationBox.show_all()

            with threadMutex:
                runThread = True
            testThread = threading.Thread(target=startCalibrationRun, args=(nodeNr, port,))
            testThread.start()                                    
        else:
            with threadMutex:
                runThread = False
            testThread.join()

    startButton[1].connect('clicked', onStartCalibration)
    calibrationBox.pack_start(startButton[0], False, False, 0)
    calibrationBox.show_all()

    return calibrationBox
