'''
Module for calibrating pwm nonlinearity
'''
# pylint: disable=duplicate-code

from ServoProjectModules.CalibrationAnalyzers.Helper import *  # pylint: disable=wildcard-import, unused-wildcard-import

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

    backlashControlSpeedScale = GuiFunctions.creatHScale(2, 0, 50, 1, getLowLev=True)
    backlashControlSpeedScale = GuiFunctions.addTopLabelTo('<b>Backlash control speed</b>\n'
        ' Lower than \'Control speed\' to avoid resonance', backlashControlSpeedScale[0]), backlashControlSpeedScale[1]
    calibrationBox.pack_start(backlashControlSpeedScale[0], False, False, 0)

    def onControlSpeedScaleChange(widget):
        contorlParameters.setMainSpeed(controlSpeedScale[1].get_value())

    controlSpeedScale[1].connect('value-changed', onControlSpeedScaleChange)

    def onAdvancedParamClicked(widget):
        nonlocal contorlParameters
        contorlParameters = GuiFunctions.openAdvancedParametersDialog(parent, contorlParameters)
        controlSpeedScale[1].set_value(contorlParameters.getMainSpeed())

    advancedParametersButton[1].connect('clicked', onAdvancedParamClicked)

    refVel = 20.0
    refPos = 0.0

    refVelScale = GuiFunctions.creatHScale(refVel, 0, 100.0, 0.1, getLowLev=True)
    refVelScale = (GuiFunctions.addTopLabelTo('<b>Max Velocity</b>\n in degrees per second', refVelScale[0]),
                    refVelScale[1])

    refPosScale = GuiFunctions.creatHScale(refPos, -90.0, 90.0, 0.1, getLowLev=True)
    refPosScale = GuiFunctions.addTopLabelTo('<b>Set position offset</b>\n in degrees', refPosScale[0]), refPosScale[1]

    startButton = GuiFunctions.createButton('Start test', getLowLev=True)

    statusLabel = GuiFunctions.createLabel('Position: - (start offset: -)\n'
                        'Velocity: -\n'
                        'Error at output: -\n'
                        'Error at motor: -\n'
                        'Control signal: -\n'
                        'Loop time: -')

    calibrationBox.pack_start(refVelScale[0], False, False, 0)
    calibrationBox.pack_start(refPosScale[0], False, False, 0)
    calibrationBox.pack_start(startButton[0], False, False, 0)
    calibrationBox.pack_start(statusLabel, False, False, 0)
    calibrationBox.show_all()

    threadMutex = threading.Lock()

    def onRefVelScaleChange(widget):
        nonlocal refVel
        with threadMutex:
            refVel = widget.get_value()

    def onRefPosScaleChange(widget):
        nonlocal refPos
        with threadMutex:
            refPos = widget.get_value()

    refVelScale[1].connect('value-changed', onRefVelScaleChange)
    refPosScale[1].connect('value-changed', onRefPosScaleChange)

    def updateStatusLabel(string):
        statusLabel.set_label(string)

    def resetGuiAfterCalibration():
        startButton[1].set_label('Start test')
        startButton[1].set_sensitive(True)
        controlSpeedScale[1].set_sensitive(True)
        advancedParametersButton[1].set_sensitive(True)

        backlashControlSpeedScale[1].set_sensitive(True)
        refPosScale[1].set_value(0.0)

    runThread = False

    def plotData(data):
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
            t = data[:, 0] - data[0, 0]
            fig = plt.figure(1)
            fig.suptitle('Position')
            plt.plot(t, data[:, 8], 'r')
            plt.plot(t, data[:, 1], 'g')

            fig = plt.figure(2)
            fig.suptitle('Velocity')
            plt.plot(t, data[:, 2])

            fig = plt.figure(3)
            fig.suptitle('Error at output')
            plt.plot(t, data[:, 3])

            fig = plt.figure(4)
            fig.suptitle('Error at motor')
            plt.plot(t, data[:, 4])

            fig = plt.figure(5)
            fig.suptitle('Control signal')
            plt.plot(t, data[:, 5])

            fig = plt.figure(6)
            fig.suptitle('Control signal over motor position')
            plt.plot(np.array(data[:, 6]), data[:, 5], 'g+')

            fig = plt.figure(7)
            fig.suptitle('Motor pos diff over motor position')
            motorPos = data[:, 6]
            plt.plot(motorPos[0:-1],
                [(d1 - d0 + 1024)%2048-1024 for d1, d0 in zip(motorPos[1:], motorPos[0:-1])], '+')

            plt.show()

    def startTestRun(nodeNr, port):
        # pylint: disable=too-many-locals, too-many-statements
        nonlocal runThread

        try:
            backlashControlSpeed = int(backlashControlSpeedScale[1].get_value())

            def initFun(servoArray):
                controlSpeed, velControlSpeed, filterSpeed, inertiaMarg = contorlParameters.getValues()
                servoArray[0].setOffsetAndScaling(360.0 / 4096.0, 0.0, 0)
                servoArray[0].setControlSpeed(controlSpeed, velControlSpeed, filterSpeed, inertiaMarg)
                servoArray[0].setBacklashControlSpeed(backlashControlSpeed, 180.0, 0.0)

            with createServoManager(nodeNr, port, dt=0.018, initFunction=initFun) as servoManager:
                t = 0.0
                pRef = 0.0
                vRef = 0.0
                doneRunning = False

                servoManager.servoArray[0].getPosition(False)
                posOffset = servoManager.servoArray[0].getPosition()
                moveHandler = SmoothMoveHandler(posOffset, 0.4)

                def sendCommandHandlerFunction(dt, servoManager):
                    nonlocal pRef
                    nonlocal vRef
                    servo = servoManager.servoArray[0]

                    with threadMutex:
                        moveHandler.set(refPos + posOffset, refVel)

                    pRef, vRef = moveHandler.getNextRef(dt)
                    servo.setReference(pRef, vRef, 0)

                out = []

                def readResultHandlerFunction(dt, servoManager):
                    nonlocal t
                    nonlocal doneRunning

                    t += dt

                    stop = False
                    with threadMutex:
                        if runThread is False:
                            stop = True

                    if stop or parent.isClosed:
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
                    out.append([time.time(), p, v,
                            error,
                            motorError,
                            u,
                            optData.minCostIndex,
                            optData.minCost,
                            refPos + posOffset,
                            vRef])

                    GLib.idle_add(updateStatusLabel,
                            f'Position: {p - posOffset:0.4f} (start offset: {posOffset:0.2f})\n'
                            f'Velocity: {v:0.4f}\n'
                            f'Error at output: {error:0.4f}\n'
                            f'Error at motor: {motorError:0.4f}\n'
                            f'Control signal: {u:0.4f}\n'
                            f'Loop time: {servo.getLoopTime()}')

                servoManager.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction)

                while not doneRunning:
                    if not servoManager.isAlive():
                        runThread = False
                        break
                    time.sleep(0.1)

                servoManager.shutdown()

                data = np.array(out)
                np.savetxt('lastTestControlLoop.txt', data)
                GLib.idle_add(plotData, data)

        except Exception as e:
            GuiFunctions.exceptionMessage(parent, e)
        finally:
            GLib.idle_add(resetGuiAfterCalibration)

    testThread = None

    def onStartTest(widget):
        nonlocal testThread
        nonlocal runThread

        if widget.get_label() == 'Start test':
            widget.set_label('Stop test')

            controlSpeedScale[1].set_sensitive(False)
            advancedParametersButton[1].set_sensitive(False)
            backlashControlSpeedScale[1].set_sensitive(False)

            calibrationBox.show_all()

            with threadMutex:
                runThread = True
            testThread = threading.Thread(target=startTestRun, args=(nodeNr, getPortFun(),))
            testThread.start()
        else:
            with threadMutex:
                runThread = False
            testThread.join()

    startButton[1].connect('clicked', onStartTest)

    return calibrationBox
