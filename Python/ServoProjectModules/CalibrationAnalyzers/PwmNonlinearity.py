'''
Module for calibrating pwm nonlinearity
'''
# pylint: disable=duplicate-code

from ServoProjectModules.CalibrationAnalyzers.Helper import *  # pylint: disable=wildcard-import, unused-wildcard-import

class QuadraticCurve:
    def __init__(self, xList, yList, index = 1):
        x0 = xList[index - 1]
        x1 = xList[index]
        x2 = xList[index + 1]
        y0 = yList[index - 1]
        y1 = yList[index]
        y2 = yList[index + 1]

        #y0 = c0 * x0**2 + c1 * x0 + c2
        #y1 = c0 * x1**2 + c1 * x1 + c2
        #y2 = c0 * x2**2 + c1 * x2 + c2
        #Y = A * C
        # ==> C = A^-1 * Y
        aMat = np.matrix([[x0**2, x0, 1],
                         [x1**2, x1, 1],
                         [x2**2, x2, 1]])

        temp = np.linalg.inv(aMat) * np.matrix([[y0], [y1], [y2]])
        self._c = list((d[0, 0] for d in temp))

        self._xCenter = x1
        self._xSpan = [x1 - x0, x2 - x1]

    def getY(self, xList):
        c0 = self._c[0]
        c1 = self._c[1]
        c2 = self._c[2]

        y = [c0 * x**2 + c1 * x + c2 for x in xList]
        return list(y)

    def addToWeightedAveraged(self, x, ywSum, wSum):
        y = self.getY(x)
        w = [math.exp(-(2 * (d - self._xCenter) /
                (self._xSpan[0] if d - self._xCenter < 0 else self._xSpan[1]))**2) for d in x]

        ywSum = [d[0] + d[1] * d[2] for d in zip(ywSum, y, w)]
        wSum = [d[0] + d[1] for d in zip(wSum, w)]
        return ywSum, wSum

class PwmNonlinearityIdentifier(PwmNonlinearityConfigHandler):
    def __init__(self, compListPwm, compList):
        self.compListPwm = []
        self.compList = []

        lastCompVal = None
        for d in zip(compListPwm, compList):
            if lastCompVal != d[1]:
                self.compListPwm.append(d[0])
                self.compList.append(d[1])
            lastCompVal = d[1]

        self.lookUpStepSize = 4

        x = np.arange(0, 1024, 1)
        ywSum = np.zeros(1024)
        wSum = np.zeros(1024)
        self.quadraticCurveList = []
        for i in range(1, len(self.compList) - 1):
            q = QuadraticCurve(self.compListPwm, self.compList, i)
            ywSum, wSum = q.addToWeightedAveraged(x, ywSum, wSum)
            self.quadraticCurveList.append(q)

        self.compListQuadInter = np.array([max(0, d[0]) / d[1] for d in zip(ywSum, wSum)])

        self.quadraticCurveScaling = 1023 / self.compListQuadInter[-1]
        self.compListQuadInter *= self.quadraticCurveScaling
        self.compList = [d * self.quadraticCurveScaling for d in self.compList]


        self.pwmNonlinearityCompLookUp = []

        for pwm in range(0, 1024, self.lookUpStepSize):
            index = -1
            for d in self.compListQuadInter[0:-1]:
                if d < pwm:
                    index += 1
                else:
                    break

            t = ((pwm - self.compListQuadInter[index]) /
                    (self.compListQuadInter[index + 1] - self.compListQuadInter[index]))
            self.pwmNonlinearityCompLookUp.append(index + t)

        super().__init__(self.pwmNonlinearityCompLookUp)

    def plotGeneratedVector(self, box):
        fig = Figure(figsize=(5, 4), dpi=100)
        ax = fig.add_subplot()

        for i, q in enumerate(self.quadraticCurveList):
            v = self.compListPwm[i + 1]
            x = np.arange(v - 150, v + 150, 10)
            y = np.array(q.getY(x)) * self.quadraticCurveScaling
            ax.plot(y, x, 'y,')
        ax.plot(range(0, 1024, self.lookUpStepSize), range(0, 1024, self.lookUpStepSize), 'k-')
        ax.plot(range(0, 1024, self.lookUpStepSize), self.pwmNonlinearityCompLookUp, 'g-')
        ax.plot(self.compList, self.compListPwm, 'c.')

        canvas = FigureCanvas(fig)
        canvas.set_size_request(600, 400)
        box.add(canvas)

        box.show_all()

def createGuiBox(parent, nodeNr, getPortFun, configFilePath, configClassName):
    # pylint: disable=too-many-locals, too-many-statements
    with open(configFilePath, 'r', encoding='utf-8') as configFile:
        configFileAsString = configFile.read()

        try:
            PwmNonlinearityIdentifier.checkForPreviousCalibration(configFileAsString, configClassName)
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

    testListBox = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
    testListBox.set_margin_start(0)

    lableBox = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
    lable = GuiFunctions.createLabel('<b>Stall current over pwm values</b>')
    lableBox.pack_start(lable, False, False, 0)
    editTestButton = GuiFunctions.createButton('Edit', width=26, getLowLev=True)
    lable.set_margin_end(4)
    editTestButton[1].set_margin_start(4)

    lableBox.pack_start(editTestButton[0], False, False, 0)
    calibrationBox.pack_start(lableBox, False, False, 0)

    scrollWin = Gtk.ScrolledWindow()
    scrollWin.set_min_content_height(100)
    scrollWin.set_margin_start(40)
    scrollWin.add(testListBox)
    calibrationBox.pack_start(scrollWin, True, True, 0)

    calculateCalibrationButton = GuiFunctions.createButton('Calculate calibration', getLowLev=True)
    calculateCalibrationButton[1].set_sensitive(False)
    calibrationBox.pack_start(calculateCalibrationButton[0], False, False, 0)

    buttons = []
    testValues = list(range(0, 1023, 100))
    testResults = ['' for v in testValues]
    if getPortFun() == '':
        testResults = ['0.0522', '0.0568', '0.0621', '0.0683', '0.0765',
                        '0.0844', '0.095', '0.172', '0.44', '0.88', '1.61']

    def getIndexOfWidget(widget):
        return [i for i, w in enumerate(buttons) if widget in (w[1], w[2])][0]

    def updateCalculateCalibrationButtonStatus():
        if testResults[0] == '':
            calculateCalibrationButton[1].set_label(
                'Data point at 0 needed to calibration')
            calculateCalibrationButton[1].set_sensitive(False)
            return

        nrOfCalibrationDataPoints = len([[d[0], d[1]] for d in zip(testValues, testResults) if d[1] != ''])
        if nrOfCalibrationDataPoints >= 3:
            calculateCalibrationButton[1].set_label('Calculate calibration')
            calculateCalibrationButton[1].set_sensitive(True)
        else:
            missingPoints = 3 - nrOfCalibrationDataPoints
            calculateCalibrationButton[1].set_label(
                    f'Minimum {missingPoints} more data point{"s" if missingPoints > 1 else ""} to calibration')
            calculateCalibrationButton[1].set_sensitive(False)

    def onEditTestValuesClicked(widget):
        # pylint: disable=too-many-locals, too-many-statements
        nonlocal buttons
        nonlocal testValues
        nonlocal testResults

        if widget is not None:
            dialog = Gtk.MessageDialog(
                    transient_for=parent,
                    flags=0,
                    message_type=Gtk.MessageType.INFO,
                    buttons=Gtk.ButtonsType.OK_CANCEL,
                    text='Edit pwm test values',
            )
            dialog.format_secondary_text(
                'Examples:\n  \'[123, 456, 789]\'\n  \'range(100, 900, 100)\''
            )
            box = dialog.get_message_area()
            testValuesEntry = Gtk.Entry()
            testValuesEntry.set_text(f'{testValues[1:]}')
            box.add(testValuesEntry)
            box.show_all()
            dialog.get_widget_for_response(Gtk.ResponseType.CANCEL).grab_focus()
            dialog.run()

            try:
                s = testValuesEntry.get_text()
                if s.find('\n') != -1:
                    raise Exception('line break not allowed')
                gen = eval(s)  # pylint: disable=eval-used
                newTestValues = [int(round(d)) for d in gen]
                if len(newTestValues) == 0 or newTestValues[0] != 0:
                    newTestValues = [0] + newTestValues
                newTestResults = ['' for d in newTestValues]
                for i, newTest in enumerate(newTestValues):
                    resultsForOldTest = [oldResult for oldTest, oldResult in zip(testValues, testResults)
                                            if newTest == oldTest]
                    if len(resultsForOldTest) > 0:
                        newTestResults[i] = resultsForOldTest[0]

                testValues = newTestValues
                testResults = newTestResults

            except Exception as e:
                errorDiaglog = Gtk.MessageDialog(
                        transient_for=parent,
                        flags=0,
                        message_type=Gtk.MessageType.ERROR,
                        buttons=Gtk.ButtonsType.OK,
                        text='Evaluation Error',
                )
                errorDiaglog.format_secondary_text(
                    f'{e!r}'
                )
                errorDiaglog.run()
                errorDiaglog.destroy()
            finally:
                dialog.destroy()

        updateCalculateCalibrationButtonStatus()

        for b in buttons:
            testListBox.remove(b[0])

        servoManager = None
        def onTestButtonPressed(widget):
            nonlocal servoManager

            index = getIndexOfWidget(widget)

            for w in buttons:
                if w[1] != widget:
                    if widget.get_active():
                        w[1].set_sensitive(False)
                    else:
                        w[1].set_sensitive(True)

            if widget.get_active():
                with open(configFilePath, 'r', encoding='utf-8') as configFile:
                    configFileAsString = configFile.read()

                    previousCalibrationDetected = False
                    try:
                        previousCalibrationDetected = PwmNonlinearityIdentifier.checkForPreviousCalibration(
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
                            widget.set_active(False)
                            return

                        configFileAsString = PwmNonlinearityIdentifier.resetPreviousCalibration(
                                configFileAsString, configClassName)
                        with open(configFilePath, 'w', encoding='utf-8') as configFile:
                            configFile.write(configFileAsString)
                            GuiFunctions.transferToTargetMessage(parent)

                        widget.set_active(False)
                        return

                pwm = int(widget.get_label())

                try:
                    servoManager = createServoManager(nodeNr, getPortFun())

                    def sendCommandHandlerFunction(dt, servoManager):
                        servoManager.servoArray[0].setOpenLoopControlSignal(pwm, True)

                    def readResultHandlerFunction(dt, servoManager):
                        return

                    def errorHandlerFunction(exception):
                        nonlocal servoManager
                        GuiFunctions.exceptionMessage(parent, exception)
                        servoManager.shutdown()
                        servoManager = None
                        widget.set_active(False)

                    servoManager.setHandlerFunctions(sendCommandHandlerFunction,
                            readResultHandlerFunction, errorHandlerFunction)
                    servoManager.start()
                except Exception as e:
                    GuiFunctions.exceptionMessage(parent, e)
                    if servoManager is not None:
                        servoManager.shutdown()
                        servoManager = None
                    widget.set_active(False)

            elif servoManager:
                servoManager.removeHandlerFunctions()
                servoManager.shutdown()
                buttons[index][2].grab_focus()
                servoManager = None

        def onResultEntryEdit(widget):
            if not onResultEntryEdit.firstLevelEvent:
                return

            index = getIndexOfWidget(widget)

            try:
                s = widget.get_text()
                newS = ''
                if s !='':
                    v = abs(float(s))
                    newS = str(v)
                    if v != float(s):
                        onResultEntryEdit.firstLevelEvent = False
                        widget.set_text(newS)
                testResults[index] = newS
            except ValueError:
                onResultEntryEdit.firstLevelEvent = False
                widget.set_text(testResults[index])
            finally:
                onResultEntryEdit.firstLevelEvent = True

                updateCalculateCalibrationButtonStatus()

        onResultEntryEdit.firstLevelEvent = True

        def onResultEntryEnter(widget):
            index = getIndexOfWidget(widget)

            if index < len(buttons) - 1:
                buttons[index + 1][1].grab_focus()
            else:
                calculateCalibrationButton[1].grab_focus()

        buttons = []
        for pwm, result in zip(testValues, testResults):
            box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
            b = Gtk.ToggleButton(label=f'{pwm}')
            b.set_margin_start(4)
            b.set_margin_end(4)
            b.set_margin_top(8)
            b.set_margin_bottom(1)
            b.set_property('width-request', 20)
            b.connect('toggled', onTestButtonPressed)
            box.pack_start(b, False, False, 0)
            e = Gtk.Entry()
            e.set_text(f'{result}')
            e.set_width_chars(3)
            e.set_margin_start(4)
            e.set_margin_end(4)
            e.set_margin_top(1)
            e.set_margin_bottom(4)
            e.connect('changed', onResultEntryEdit)
            e.connect('activate', onResultEntryEnter)
            box.pack_start(e, False, False, 0)
            buttons.append((box, b, e))


        for b in buttons:
            testListBox.pack_start(b[0], False, False, 0)

        testListBox.show_all()

    onEditTestValuesClicked(None)
    editTestButton[1].connect('clicked', onEditTestValuesClicked)

    def onCalculateCalibrationButtonClicked(widget):
        data = [[d[0], d[1]] for d in zip(testValues, testResults) if d[1] != '']
        compListPwm = [int(d[0]) for d in data]
        compList = [float(d[1]) - float(data[0][1]) for d in data]

        pwmNonlinearityIdentifier = PwmNonlinearityIdentifier(compListPwm, compList)

        dialog = Gtk.MessageDialog(
                transient_for=parent,
                flags=0,
                message_type=Gtk.MessageType.INFO,
                buttons=Gtk.ButtonsType.YES_NO,
                text='Pwm linearity calibration done!',
        )
        dialog.format_secondary_text(
            'Should the configuration be updated with the new data?'
        )
        pwmNonlinearityIdentifier.plotGeneratedVector(dialog.get_message_area())
        dialog.get_widget_for_response(Gtk.ResponseType.YES).grab_focus()
        response = dialog.run()
        dialog.destroy()

        if response == Gtk.ResponseType.YES:
            with open(configFilePath, 'r', encoding='utf-8') as configFile:
                configFileAsString = configFile.read()

                configFileAsString = pwmNonlinearityIdentifier.writeLinearizationFunctionToConfigFileString(
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
                'Please past in the linearization function manually'
            )
            box = dialog.get_message_area()
            funEntry = Gtk.Entry()
            funEntry.set_text(pwmNonlinearityIdentifier.getLinearizationFunction())
            box.add(funEntry)
            box.show_all()
            response = dialog.run()
            dialog.destroy()

    calculateCalibrationButton[1].connect('clicked', onCalculateCalibrationButtonClicked)

    calibrationBox.show_all()
    return calibrationBox
