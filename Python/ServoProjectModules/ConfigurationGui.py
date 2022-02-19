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

def openCreateConfigDialog(parent, configs):
    dialog = Gtk.MessageDialog(
            transient_for=parent,
            flags=0,
            message_type=Gtk.MessageType.OTHER,
            buttons=Gtk.ButtonsType.OK_CANCEL,
            text='Create new configuration',
    )
    dialog.format_secondary_text(
        ""
    )
    box = dialog.get_message_area()
    createButton = dialog.get_widget_for_response(Gtk.ResponseType.OK)
    createButton.set_label('Create')
    createButton.set_sensitive(False)

    def onNewConfiNameEntryChange(widget):
        newName = widget.get_text()
        newFilePath = parent.ArduinoSketchPath + "/config/" + newName

        if (not os.path.isdir(newFilePath) and
            len(newName) > 2 and
            newName.find('.') == len(newName) - 2 and 
            newName[-1] == 'h' and
            newName.find('/') == -1 and
            newName.find('\\') == -1):

            if os.path.isfile(newFilePath):
                createButton.set_label('Overwrite')
            else:
                createButton.set_label('Create')
            createButton.set_sensitive(True)
        else:
            createButton.set_sensitive(False)
            createButton.set_label('Create')

    box1 = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
    templateConfigCombo = GuiFunctions.creatComboBox(configs[0], configs, getLowLev=True)
    templateConfigCombo = GuiFunctions.addTopLabelTo('<b>From template</b>', templateConfigCombo[0]), templateConfigCombo[1]
    box1.pack_start(templateConfigCombo[0], False, False, 0)
    newNodeNrSpinButton = GuiFunctions.creatSpinButton(1, 1, 255, 1, getLowLev=True)
    newNodeNrSpinButton = GuiFunctions.addTopLabelTo('<b>With new node number</b>', newNodeNrSpinButton[0]), newNodeNrSpinButton[1]
    box1.pack_start(newNodeNrSpinButton[0], False, False, 0)
    box.pack_start(box1, False, False, 0)

    gearingEntry = GuiFunctions.createEntry('', getLowLev=True)
    gearingEntry = GuiFunctions.addTopLabelTo('<b>Gear ratio</b>\n Ex: 10 / 1 * 11 / 62 * 14 / 48 * 13 / 45 * 1 / 42', gearingEntry[0]), gearingEntry[1]
    box.pack_start(gearingEntry[0], False, False, 0)

    potentiometerRangeSpinButton = GuiFunctions.creatSpinButton(220, -360, 360, 1, getLowLev=True)

    encoderTypes = ['Potentiometer', 'Magnetic']

    def onEncoderTypeComboChange(widget):
        if widget.get_active() == 0:
            potentiometerRangeSpinButton[0].show_all()
        else:
            potentiometerRangeSpinButton[0].hide()

    box2 = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
    encoderTypeCombo = GuiFunctions.creatComboBox(encoderTypes[0], encoderTypes, onChangeFun=onEncoderTypeComboChange, getLowLev=True)
    encoderTypeCombo = GuiFunctions.addTopLabelTo('<b>Output encoder type</b>', encoderTypeCombo[0]), encoderTypeCombo[1]
    box2.pack_start(encoderTypeCombo[0], False, False, 0)
    potentiometerRangeSpinButton = GuiFunctions.addTopLabelTo('<b>Potentiometer range</b>', potentiometerRangeSpinButton[0]), potentiometerRangeSpinButton[1]
    box2.pack_start(potentiometerRangeSpinButton[0], False, False, 0)
    box.pack_start(box2, False, False, 0)

    newConfigurationNameEntry = GuiFunctions.createEntry('', onEdit=onNewConfiNameEntryChange, getLowLev=True)
    newConfigurationNameEntry = GuiFunctions.addTopLabelTo('<b>Enter new name</b>\n (*.h)', newConfigurationNameEntry[0]), newConfigurationNameEntry[1]
    box.pack_start(newConfigurationNameEntry[0], False, False, 0)

    box.show_all()

    def getTemplateConfig():
        templateStr = templateConfigCombo[1].get_model()[templateConfigCombo[1].get_active()][0]
        descStartIndex = templateStr.find(' : ')
        if descStartIndex != -1:
            return templateStr[0: descStartIndex], templateStr[descStartIndex + 3:]
        return '', ''

    def getTemplateConfigClassString(configName, configClassName):
        configFilePath = parent.ArduinoSketchPath + "/config/" + configName
        configFileAsString = ''

        with open(configFilePath, "r") as configFile:
            configFileAsString = configFile.read()

        configClassString = Helper.getConfigClassString(configFileAsString, configClassName)

        return configClassString

    def onTemplateConfigComboChange(widget):
        templateConfig = getTemplateConfig()
        configName = templateConfig[0]
        configClassName = templateConfig[1]
        configClassString = getTemplateConfigClassString(configName, configClassName)

        try:
            gearingStr = Helper.getConfiguredGearRatio(configClassString)
            gearingEntry[1].set_text(gearingStr)

            magneticEncoder, unitsPerRev = Helper.getConfiguredOutputEncoderData(configClassString)
            encoderTypeCombo[1].set_active(1 if magneticEncoder else 0)
            potentiometerRangeSpinButton[1].set_value(round(unitsPerRev / 4096 * 360))

            gearingEntry[0].show()
            encoderTypeCombo[0].show()
            potentiometerRangeSpinButton[0].show()
        except Exception as e:
            gearingEntry[0].hide()
            encoderTypeCombo[0].hide()
            potentiometerRangeSpinButton[0].hide()

    onTemplateConfigComboChange(None)
    templateConfigCombo[1].connect('changed', onTemplateConfigComboChange)

    createdConfigName = ''
    if dialog.run() == Gtk.ResponseType.OK:
        templateConfig = getTemplateConfig()
        configName = templateConfig[0]
        configClassName = templateConfig[1]
        configClassString = getTemplateConfigClassString(configName, configClassName)

        if gearingEntry[0].get_visible():
            configClassString = Helper.setConfiguredGearRatio(configClassString, gearingEntry[1].get_text())

            magneticEncoder = encoderTypeCombo[1].get_active() == 1
            unitsPerRev = potentiometerRangeSpinButton[1].get_value() / 360 * 4096
            configClassString = Helper.setConfiguredOutputEncoderData(configClassString, magneticEncoder, unitsPerRev)

        newConfigFileAsString = Helper.newConfigFileAsString(configClassString, newNodeNrSpinButton[1].get_value(), configClassName)
        if newConfigFileAsString != '':
            configName = newConfigurationNameEntry[1].get_text()
            newConfigFilePath = parent.ArduinoSketchPath + "/config/" + configName
            with open(newConfigFilePath, "w") as configFile:
                configFile.write(newConfigFileAsString)
                createdConfigName = configName

    dialog.destroy()

    return createdConfigName

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
            configs = ['']
            for c in self.getConfigurations():
                configs.append(c)
            configs.sort()
            configs = [c for c in configs if c != 'default.h']

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

        def onCreateConfigClicked(widget):
            configs = []
            for c in self.getConfigurations():
                nodeNrList, configClassNames = self.getNodeNrAndClassNames(c)
                for className in configClassNames:
                    configs.append(f'{c} : {className}')
            configs.sort()

            newConfigName = openCreateConfigDialog(self, configs)
            if newConfigName != '':
                loadConfigs(None, None)
                selectActiveConfig(newConfigName)

        def onDeleteConfigClicked(widget):
            configName = ''
            activeIter = activeConfigCombo[1].get_active_iter()
            if activeIter is not None:
                model = activeConfigCombo[1].get_model()
                configName = model[activeIter][0]

            if configName == '':
                return

            configFilePath = self.ArduinoSketchPath + "/config/" + configName
            os.remove(configFilePath)
            selectActiveConfig('')
            loadConfigs(None, None)

        box2 = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
        box2.add(activeConfigCombo[0])
        activeConfigCombo = box2, activeConfigCombo[1]

        creatNewConfigButton = GuiFunctions.createButton('Create new', onClickFun=onCreateConfigClicked, width=10, getLowLev=True)
        creatNewConfigButton[1].set_margin_start(0)
        creatNewConfigButton[1].set_margin_end(10)
        creatNewConfigButton[1].set_margin_top(10)
        creatNewConfigButton[1].set_margin_bottom(8)
        box2.add(creatNewConfigButton[0])

        deleteConfigButton = GuiFunctions.createButton('Delete', onClickFun=onDeleteConfigClicked, width=10, getLowLev=True)
        deleteConfigButton[1].set_margin_start(0)
        deleteConfigButton[1].set_margin_end(10)
        deleteConfigButton[1].set_margin_top(10)
        deleteConfigButton[1].set_margin_bottom(8)
        deleteConfigButton[1].set_sensitive(False)
        box2.add(deleteConfigButton[0])

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
                    deleteConfigButton[1].set_sensitive(False)
                    transferToTargetButton[1].set_sensitive(False)
                    items = Gtk.ListStore(str)
                    for i, name in enumerate(['']):
                        n = []
                        n.append(name)
                        items.append(n)

                    activeNodeNrCombo[1].set_model(items)
                    activeNodeNrCombo[1].set_active(0)
                    return

                deleteConfigButton[1].set_sensitive(True)
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
