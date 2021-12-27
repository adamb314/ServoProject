from ServoProjectModules.CalibrationAnalyzers.Helper import *

def createGuiBox(parent, nodeNr, port, configFilePath, configClassName, advancedMode):
    calibrationBox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
    calibrationBox.set_margin_start(40)

    controlSpeedScale = GuiFunctions.creatHScale(14, 0, 100, 1, getLowLev=True)
    controlSpeedScale = GuiFunctions.addTopLabelTo('<b>Control speed</b>', controlSpeedScale[0]), controlSpeedScale[1]
    calibrationBox.pack_start(controlSpeedScale[0], False, False, 0)

    if advancedMode == True:
        velControlSpeedScale = GuiFunctions.creatHScale(14 * 4, 0, 100 * 4, 4, getLowLev=True)
        velControlSpeedScale = GuiFunctions.addTopLabelTo('<b>Velocity control speed</b>', velControlSpeedScale[0]), velControlSpeedScale[1]
        calibrationBox.pack_start(velControlSpeedScale[0], False, False, 0)

        filterSpeedScale = GuiFunctions.creatHScale(14 * 32, 0, 100 * 32, 32, getLowLev=True)
        filterSpeedScale = GuiFunctions.addTopLabelTo('<b>Filter speed</b>', filterSpeedScale[0]), filterSpeedScale[1]
        calibrationBox.pack_start(filterSpeedScale[0], False, False, 0)

    backlashControlSpeedScale = GuiFunctions.creatHScale(0, 0, 50, 1, getLowLev=True)
    backlashControlSpeedScale = GuiFunctions.addTopLabelTo('<b>Backlash control speed</b>', backlashControlSpeedScale[0]), backlashControlSpeedScale[1]
    calibrationBox.pack_start(backlashControlSpeedScale[0], False, False, 0)

    testVel = 0

    testVelScale = GuiFunctions.creatHScale(0.0, -1.000, 1.000, 0.0001, getLowLev=True)
    testVelScale = GuiFunctions.addTopLabelTo('<b>Velocity</b>\n in radians per second', testVelScale[0]), testVelScale[1]
    startButton = GuiFunctions.createButton('Start test', getLowLev=True)

    positionOffsetScale = GuiFunctions.creatHScale(0.0, -1.0, 1.0, 0.001, getLowLev=True)
    positionOffsetScale = GuiFunctions.addTopLabelTo('<b>Position offset</b>\n in radians', positionOffsetScale[0]), positionOffsetScale[1]
    calibrationBox.pack_start(positionOffsetScale[0], False, False, 0)

    def onTestVelScaleChange(widget):
        nonlocal threadMutex
        nonlocal testVel

        with threadMutex:
            testVel = widget.get_value()

    testVelScale[1].connect('value-changed', onTestVelScaleChange)

    threadMutex = threading.Lock()

    def resetGuiAfterCalibration():
        nonlocal startButton
        nonlocal controlSpeedScale
        nonlocal velControlSpeedScale
        nonlocal filterSpeedScale
        nonlocal backlashControlSpeedScale
        nonlocal positionOffsetScale

        startButton[1].set_label('Start test')
        startButton[1].set_sensitive(True)
        controlSpeedScale[1].set_sensitive(True)
        if advancedMode == True:
            velControlSpeedScale[1].set_sensitive(True)
            filterSpeedScale[1].set_sensitive(True)

        backlashControlSpeedScale[1].set_sensitive(True)
        positionOffsetScale[1].set_sensitive(True)

    runThread = False

    def handleResults(data):
        np.savetxt('compData_new.txt', data, delimiter=',')
        #data = np.loadtxt('compData4000Hz.txt', delimiter=',')
        plt.figure(1)
        plt.plot(data[:, 0], data[:, 4])

        plt.figure(2)
        plt.plot(data[:, 4], data[:, 2], 'g-')
        plt.plot(data[:, 4], data[:, 2], 'r+')

        plt.figure(3)
        plt.plot(data[:, 0], data[:, 3])

        plt.figure(4)
        plt.plot(data[:, 0], data[:, 5], 'g-')
        plt.plot(data[:, 0], data[:, 5], 'r+')

        plt.figure(5)
        plt.plot(data[:, 4], data[:, 3] / (10.0 / 1 * 11.0 / 62 * 14.0 / 48 * 13.0 / 45 * 1.0 / 42) / 2 / pi * 512, 'g-')
        plt.plot(data[:, 4], data[:, 3] / (10.0 / 1 * 11.0 / 62 * 14.0 / 48 * 13.0 / 45 * 1.0 / 42) / 2 / pi * 512, 'r+')

        plt.figure(6)
        plt.plot(data[:, 4], data[:, 6], 'g-')
        plt.plot(data[:, 4], data[:, 6], 'r+')

        plt.figure(7)
        plt.plot(data[:, 0], data[:, 7], 'g')
        
        plt.figure(8)
        plt.plot(data[:, 0], data[:, 1])
        plt.show()

        samplesList = []
        for i in range (0, 512):
            samplesList.append([])

        for d in zip(data[:, 4], data[:, 5]):
            for i in range(-8, 9):
                i = int(d[0] / 4) + i
                if i >= len(samplesList):
                    i -= len(samplesList)
                samplesList[i].append(d[1])

        unsortedList = samplesList[:]
        samplesList = []
        for d in unsortedList:
            samplesList.append(sorted(d))

        forcePos = np.zeros(len(samplesList))
        forceNeg = np.zeros(len(samplesList))

        for i, d in enumerate(samplesList):
            l = int(len(d) * 0.1)
            forcePos[i] = d[-l-1]
            forceNeg[i] = d[l]

        print(intArrayToString(forcePos))
        print(intArrayToString(forceNeg))
        plt.plot(np.array(data[:, 4]) / 4, data[:, 5], 'g+')
        plt.plot(forcePos, 'r')
        plt.plot(forceNeg, 'b')
        plt.show()

    def startTestRun(nodeNr, port):
        nonlocal runThread
        nonlocal threadMutex
        nonlocal testVel

        controlSpeed = int(controlSpeedScale[1].get_value())
        velControlSpeed = controlSpeed * 4
        filterSpeed = controlSpeed * 32
        if advancedMode == True:
            velControlSpeed = int(round(velControlSpeedScale[1].get_value() / 4.0)) * 4
            velControlSpeedScale[1].set_value(velControlSpeed)
            filterSpeed = int(round(filterSpeedScale[1].get_value() / 32.0)) * 32
            filterSpeedScale[1].set_value(filterSpeed)
        backlashControlSpeed = int(backlashControlSpeedScale[1].get_value())

        positionOffset = positionOffsetScale[1].get_value()

        try:
            def initFun(robot):
                robot.servoArray[0].setControlSpeed(controlSpeed, velControlSpeed, filterSpeed)
                robot.servoArray[0].setBacklashControlSpeed(backlashControlSpeed, 3.0, 0.0)

            robot = createRobot(nodeNr, port, dt=0.018, initFunction=initFun)

            t = 0.0
            doneRunning = False

            pos = robot.servoArray[0].getPosition() + positionOffset

            def sendCommandHandlerFunction(dt, robot):
                nonlocal t
                nonlocal threadMutex
                nonlocal pos
                nonlocal testVel

                servo = robot.servoArray[0]

                vel = 0
                with threadMutex:
                    vel = testVel

                servo.setReference(pos, vel, 0)

                pos += dt * vel

            out = []

            def readResultHandlerFunction(dt, robot):
                nonlocal t
                nonlocal runThread
                nonlocal doneRunning
                nonlocal pos

                t += dt

                stop = False
                with threadMutex:
                    if runThread == False:
                        stop = True

                if stop or parent.isClosed:
                    robot.removeHandlerFunctions()
                    doneRunning = True
                    return

                servo = robot.servoArray[0]
                optData = servo.getOpticalEncoderChannelData()
                out.append([time.time(),
                        servo.getPosition(True),
                        servo.getVelocity(),
                        servo.getControlError(False),
                        optData.minCostIndex,
                        servo.getControlSignal(),
                        optData.minCost,
                        servo.getLoopTime(),
                        pos])

            robot.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction);

            while not doneRunning:
                time.sleep(0.1)

            robot.shutdown()

            data = np.array(out)
            GLib.idle_add(handleResults, data)

        except Exception as e:
            print(format(e))

        GLib.idle_add(resetGuiAfterCalibration)

    testThread = None

    def onStartCalibration(widget):
        nonlocal testThread
        nonlocal threadMutex
        nonlocal runThread

        nonlocal calibrationBox
        nonlocal controlSpeedScale
        nonlocal velControlSpeedScale
        nonlocal filterSpeedScale
        nonlocal backlashControlSpeedScale
        nonlocal positionOffsetScale

        if widget.get_label() == 'Start test':
            widget.set_label('Stop test')

            controlSpeedScale[1].set_sensitive(False)
            if advancedMode == True:
                velControlSpeedScale[1].set_sensitive(False)
                filterSpeedScale[1].set_sensitive(False)
            backlashControlSpeedScale[1].set_sensitive(False)
            positionOffsetScale[1].set_sensitive(False)

            calibrationBox.show_all()

            with threadMutex:
                runThread = True
            testThread = threading.Thread(target=startTestRun, args=(nodeNr, port,))
            testThread.start()                                    
        else:
            with threadMutex:
                runThread = False
            testThread.join()

    calibrationBox.pack_start(testVelScale[0], False, False, 0)
    startButton[1].connect('clicked', onStartCalibration)
    calibrationBox.pack_start(startButton[0], False, False, 0)
    calibrationBox.show_all()

    return calibrationBox
