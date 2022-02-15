import os
import serial.tools.list_ports
import webbrowser
import re
import threading
import ServoProjectModules.GuiHelper as GuiFunctions
from ServoProjectModules.GuiHelper import GLib, Gtk
import ServoProjectModules.CalibrationAnalyzers.OpticalEncoder as OpticalEncoderAnalyzer
import ServoProjectModules.CalibrationAnalyzers.PwmNonlinearity as PwmNonlinearityAnalyzer
import ServoProjectModules.CalibrationAnalyzers.SystemIdentification as SystemIdentificationAnalyzer
import ServoProjectModules.CalibrationAnalyzers.OutputEncoder as OutputEncoderAnalyzer
import ServoProjectModules.CalibrationAnalyzers.TestControlLoop as TestControlLoopAnalyzer
import ServoProjectModules.CalibrationAnalyzers.Helper as Helper
import ServoProjectModules.ArduinoManager as ArduinoManager

class GuiWindow(Gtk.Window):
    def __init__(self, ArduinoSketchPath):
        Gtk.Window.__init__(self, title="Servo configuration", default_height=900, default_width=800)

        self.ArduinoSketchPath = ArduinoSketchPath
        self.vboxMain = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        self.scrollWin = Gtk.ScrolledWindow()
        self.scrollWin.add(self.vboxMain)
        self.windowBox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        self.windowBox.pack_start(self.scrollWin, True, True, 0)
        self.add(self.windowBox)

        byLabel = Gtk.Label(label='by <b>Adam Bäckström</b>')
        byLabel.set_use_markup(True)
        byLabel.set_margin_start(10)
        byLabel.set_margin_end(10)
        byLabel.set_margin_top(8)
        byLabel.set_margin_bottom(10)
        byLabel.set_xalign(0.0)

        statusBarBox = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
        statusBarBox.pack_start(byLabel, False, False, 0)
        self.windowBox.pack_start(Gtk.Separator(orientation=Gtk.Orientation.HORIZONTAL), False, False, 0)
        self.windowBox.pack_start(statusBarBox, False, False, 0)

        def gotoToStaring():
            webbrowser.open('https://github.com/adamb314/ServoProject/stargazers')
            with open('../.thankyou', 'w') as f:
                f.write('Thankyou for staring the project!')

        def considerStaringMessage():
            try:
                with open('../.thankyou', 'r') as f:
                    data = f.read()
                    if data == 'Thankyou for staring the project!':
                        return
            except Exception as e:
                pass

            dialog = Gtk.MessageDialog(
                    transient_for=self,
                    flags=0,
                    message_type=Gtk.MessageType.INFO,
                    buttons=Gtk.ButtonsType.YES_NO,
                    text='Consider staring',
            )
            dialog.format_secondary_text(
                "Has this project been useful for you?\nConsider giving it a \N{glowing star} on github"
            )
            response = dialog.run()
            dialog.destroy()
            if response == Gtk.ResponseType.YES:
                gotoToStaring()

        def starButtonClicked(widget):
            gotoToStaring()

        starButton = Gtk.Button(label=' \N{glowing star} Like')
        starButton.connect("clicked", starButtonClicked)
        starButton.set_margin_top(2)
        starButton.set_margin_bottom(2)
        starButton.set_margin_end(2)
        statusBarBox.pack_end(starButton, False, False, 0)

        self.isClosed = False

        def closeEvent(widget, event):
            considerStaringMessage()
            self.isClosed = True
        self.connect('delete-event', closeEvent)

        box0 = GuiFunctions.createLabelBox('<big><b>Active configuration</b></big>')
        self.vboxMain.pack_start(box0, False, False, 20)
        box1 = GuiFunctions.createLabelBox('<big><b>Calibration</b></big>')
        self.vboxMain.pack_start(box1, False, False, 20)

        activeConfigCombo = GuiFunctions.creatComboBox('', [''], getLowLev=True)
        def loadConfigs(widget, event):
            nonlocal activeConfigCombo
            configs = ['']
            for c in self.getConfigurations():
                configs.append(c)
            configs.sort()

            activeIndex = 0
            items = activeConfigCombo[1].get_model()
            currentItem = items[activeConfigCombo[1].get_active()][0]
            items = Gtk.ListStore(str)
            for i, name in enumerate(configs):
                n = []
                n.append(name)
                items.append(n)

                if currentItem == name:
                    activeIndex = i

            activeConfigCombo[1].set_model(items)
            activeConfigCombo[1].set_active(activeIndex)

        def selectActiveConfig(configName):
            items = activeConfigCombo[1].get_model()
            l = [i for i, d in enumerate(items) if d[0] == configName]
            if len(l) > 0:
                activeConfigCombo[1].set_active(l[0])

        loadConfigs(None, None)
        activeConfigCombo[0].connect('enter-notify-event', loadConfigs)
        activeConfigCombo[1].connect('focus', loadConfigs)

        activeNodeNrCombo = GuiFunctions.creatComboBox('', [''], getLowLev=True)

        activeComPortCombo = GuiFunctions.creatComboBox('', [''], getLowLev=True)
        ignoreComPortComboChangeEvent = False
        def loadComPorts(widget, event):
            nonlocal ignoreComPortComboChangeEvent
            ports = []

            for port, desc, hwid in serial.tools.list_ports.comports():
                    ports.append(str(port) + ': ' + str(desc))
            ports.sort()
            ports.append(': Simulation')

            activeIndex = 0
            items = activeComPortCombo[1].get_model()
            currentItem = items[activeComPortCombo[1].get_active()][0]
            items = Gtk.ListStore(str)
            for i, name in enumerate(ports):
                n = []
                n.append(name)
                items.append(n)

                if currentItem == name:
                    activeIndex = i

            ignoreComPortComboChangeEvent = True
            activeComPortCombo[1].set_model(items)
            activeComPortCombo[1].set_active(activeIndex)
            ignoreComPortComboChangeEvent = False

        loadComPorts(None, None)
        activeComPortCombo[0].connect('enter-notify-event', loadComPorts)
        activeComPortCombo[1].connect('focus', loadComPorts)

        transferToTargetButton = GuiFunctions.createButton('Transfer to target', width=50, getLowLev=True)
        transferToTargetButton[1].set_margin_start(0)
        transferToTargetButton[1].set_margin_end(10)
        transferToTargetButton[1].set_margin_top(10)
        transferToTargetButton[1].set_margin_bottom(8)
        transferToTargetButton[1].set_sensitive(False)
        box3 = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
        box3.add(activeComPortCombo[0])
        box3.add(transferToTargetButton[0])
        activeComPortCombo = box3, activeComPortCombo[1]
        self.transferToTargetButton = transferToTargetButton[1]

        activeConfigCombo = GuiFunctions.addTopLabelTo('<b>Configuration</b>', activeConfigCombo[0]), activeConfigCombo[1]
        box0.pack_start(activeConfigCombo[0], False, False, 0)

        activeComPortCombo = GuiFunctions.addTopLabelTo('<b>COM port</b>', activeComPortCombo[0]), activeComPortCombo[1]
        box0.pack_start(activeComPortCombo[0], False, False, 0)

        activeNodeNrCombo = GuiFunctions.addTopLabelTo('<b>Select node number</b>', activeNodeNrCombo[0]), activeNodeNrCombo[1]

        calibrationCombo = None

        def onNodeNrChange(widget):
            if calibrationCombo and not ignoreComPortComboChangeEvent:
                calibrationCombo[1].set_active(0)

        activeNodeNrCombo[1].connect('changed', onNodeNrChange)

        def getComPortFromCombo():
            portString = activeComPortCombo[1].get_model()[activeComPortCombo[1].get_active()][0]
            descStartIndex = portString.find(':')
            if descStartIndex != -1:
                return portString[0: descStartIndex]
            return ''

        def onTranferToTarget(widget):
            t = None

            dialog = Gtk.MessageDialog(
                    transient_for=self,
                    flags=0,
                    message_type=Gtk.MessageType.INFO,
                    buttons=Gtk.ButtonsType.NONE,
                    text='Transferring to target...',
            )
            dialog.format_secondary_text('')
            dialog.connect('delete-event', lambda w, e : t.join())

            def resetAfterTransfer(ok):
                t.join()
                dialog.destroy()

                resultDialog = Gtk.MessageDialog(
                        transient_for=self,
                        flags=0,
                        message_type=Gtk.MessageType.INFO if ok else Gtk.MessageType.ERROR,
                        buttons=Gtk.ButtonsType.OK,
                        text='Transfer Complete' if ok else 'Transfer Failed!',
                )
                resultDialog.format_secondary_text(
                    '' if ok else 'Please transfer to target manually'
                )
                resultDialog.run()
                resultDialog.destroy()

            def transferThreadRun():
                ok = ArduinoManager.transfer(getComPortFromCombo())
                GLib.idle_add(resetAfterTransfer, ok)

            t = testThread = threading.Thread(target=transferThreadRun)
            t.start()

            dialog.run()

        transferToTargetButton[1].connect('clicked', onTranferToTarget)

        lastActiveConfig = ''
        calibrationBox = None
        def onActiveConfigChange(widget):
            nonlocal lastActiveConfig
            nonlocal calibrationCombo
            nonlocal activeNodeNrCombo
            nonlocal calibrationBox

            activeIter = widget.get_active_iter()
            if activeIter is not None:
                model = widget.get_model()
                configName = model[activeIter][0]


                if configName == lastActiveConfig:
                    return

                lastActiveConfig = configName

                if calibrationCombo != None:
                    box1.remove(calibrationCombo[0])
                    if calibrationBox != None:
                        box1.remove(calibrationBox)

                if configName == '':
                    transferToTargetButton[1].set_sensitive(False)
                    items = Gtk.ListStore(str)
                    for i, name in enumerate(['']):
                        n = []
                        n.append(name)
                        items.append(n)

                    activeNodeNrCombo[1].set_model(items)
                    activeNodeNrCombo[1].set_active(0)
                    return

                transferToTargetButton[1].set_sensitive(True)

                self.setActiveConfig(configName)

                nodeNrList, configClassNames = self.getNodeNrAndClassNames(configName)

                if len(nodeNrList) == 0 or len(configClassNames) == 0:
                    box1.remove(activeNodeNrCombo[0])
                    supportedCalibrationOptions = ['']

                else:
                    items = Gtk.ListStore(str)
                    for i, name in enumerate(nodeNrList):
                        n = []
                        n.append(name)
                        items.append(n)

                    activeNodeNrCombo[1].set_model(items)
                    activeNodeNrCombo[1].set_active(0)

                    if len(items) == 1:
                        box1.remove(activeNodeNrCombo[0])
                    else:
                        box1.remove(activeNodeNrCombo[0])
                        box1.pack_start(activeNodeNrCombo[0], False, False, 0)

                    supportedCalibrationOptions = ['',
                            'Pwm nonlinearity',
                            'Optical encoder',
                            'System identification',
                            'Output encoder calibration',
                            'Test control loop',
                            'Test control loop (Advanced)']

                def getNodeNrFromCombo(nodeNrCombo):
                    nodeNr = nodeNrCombo.get_model()[nodeNrCombo.get_active()][0]
                    nodeNr = int(nodeNr)

                    return nodeNr

                def getConfigClassNameFromCombo(nodeNrCombo):
                    return configClassNames[nodeNrCombo.get_active()]

                def onCaribrationTypeChange(widget):
                    nonlocal calibrationBox
                    nonlocal box1
                    calibrationType = ''
                    activeIter = widget.get_active_iter()
                    if activeIter is not None:
                        model = widget.get_model()
                        calibrationType = model[activeIter][0]

                    if calibrationBox != None:
                        box1.remove(calibrationBox)

                    configFilePath = self.ArduinoSketchPath + "/config/" + configName

                    if calibrationType == '':
                        calibrationBox = None

                    elif calibrationType == 'Pwm nonlinearity':
                        nodeNr = getNodeNrFromCombo(activeNodeNrCombo[1])
                        getPortFun = getComPortFromCombo
                        configClassName = getConfigClassNameFromCombo(activeNodeNrCombo[1])
                        calibrationBox = PwmNonlinearityAnalyzer.createGuiBox(self, nodeNr, getPortFun, configFilePath, configClassName)

                    elif calibrationType == 'Optical encoder':
                        nodeNr = getNodeNrFromCombo(activeNodeNrCombo[1])
                        getPortFun = getComPortFromCombo
                        configClassName = getConfigClassNameFromCombo(activeNodeNrCombo[1])
                        calibrationBox = OpticalEncoderAnalyzer.createGuiBox(self, nodeNr, getPortFun, configFilePath, configClassName)

                    elif calibrationType == 'System identification':
                        nodeNr = getNodeNrFromCombo(activeNodeNrCombo[1])
                        getPortFun = getComPortFromCombo
                        configClassName = getConfigClassNameFromCombo(activeNodeNrCombo[1])
                        calibrationBox = SystemIdentificationAnalyzer.createGuiBox(self, nodeNr, getPortFun, configFilePath, configClassName)

                    elif calibrationType == 'Output encoder calibration':
                        nodeNr = getNodeNrFromCombo(activeNodeNrCombo[1])
                        getPortFun = getComPortFromCombo
                        configClassName = getConfigClassNameFromCombo(activeNodeNrCombo[1])
                        calibrationBox = OutputEncoderAnalyzer.createGuiBox(self, nodeNr, getPortFun, configFilePath, configClassName)

                    elif calibrationType == 'Test control loop' or calibrationType == 'Test control loop (Advanced)':
                        advancedMode = False
                        if calibrationType == 'Test control loop (Advanced)':
                            advancedMode = True
                        
                        nodeNr = getNodeNrFromCombo(activeNodeNrCombo[1])
                        getPortFun = getComPortFromCombo
                        configClassName = getConfigClassNameFromCombo(activeNodeNrCombo[1])
                        calibrationBox = TestControlLoopAnalyzer.createGuiBox(self, nodeNr, getPortFun, configFilePath, configClassName, advancedMode)

                    if calibrationBox != None:
                        box1.pack_start(calibrationBox, False, False, 0)

                calibrationCombo = GuiFunctions.creatComboBox('', supportedCalibrationOptions, onCaribrationTypeChange, getLowLev=True)
                calibrationCombo = GuiFunctions.addTopLabelTo('<b>Supported calibrations</b>', calibrationCombo[0]), calibrationCombo[1]
                box1.pack_start(calibrationCombo[0], False, False, 0)
                box1.show_all()


        activeConfigCombo[1].connect('changed', onActiveConfigChange)

    def getConfigurations(self):
        configs = []
        path = 'config'
        for filename in os.listdir(path):
            ipath = path + '/' + filename
            if not os.path.isdir(ipath) and not filename == 'config.h' and filename[-2:] == '.h':
                configs.append(filename)

        configs.sort()
        return configs

    def setActiveConfig(self, configName):
        with open(self.ArduinoSketchPath + "/config/config.h", "w") as configFile:
            configFile.write("#include \"" + configName + "\"\n")

    def getNodeNrAndClassNames(self, configName):
        nodeNrList = []
        configClassNames = []

        configFilePath = self.ArduinoSketchPath + "/config/" + configName
        configFileAsString = ''

        with open(configFilePath, "r") as configFile:
            configFileAsString = configFile.read()

            tempConfigFileAsString = configFileAsString
            dcServoPattern = re.compile(r'make_unique\s*<\s*DCServoCommunicationHandler\s*>\s*\(([0-9]*)\s*,\s*createDCServo\s*<\s*(\w+)\s*>')
            temp = dcServoPattern.search(tempConfigFileAsString)
            while temp != None:
                nodeNrList.append(temp.group(1))
                configClassNames.append(temp.group(2))
                tempConfigFileAsString = tempConfigFileAsString[temp.end(1):]
                temp = dcServoPattern.search(tempConfigFileAsString)

        return nodeNrList, configClassNames

    def setFocusOnTranferButton(self):
        self.transferToTargetButton.grab_focus()
