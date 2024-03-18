'''
Module for ArduinoSketch configuration gui
'''

import os
import webbrowser
import re
import time
import threading
import serial.tools.list_ports
import ServoProjectModules.GuiHelper as GuiFunctions
from ServoProjectModules.GuiHelper import GLib, Gtk
import ServoProjectModules.CalibrationAnalyzers.OpticalEncoder as OpticalEncoderAnalyzer
import ServoProjectModules.CalibrationAnalyzers.PwmNonlinearity as PwmNonlinearityAnalyzer
import ServoProjectModules.CalibrationAnalyzers.SystemIdentification as SystemIdentificationAnalyzer
import ServoProjectModules.CalibrationAnalyzers.OutputEncoder as OutputEncoderAnalyzer
import ServoProjectModules.CalibrationAnalyzers.MotorCoggingTorque as MotorCoggingTorqueAnalyzer
import ServoProjectModules.CalibrationAnalyzers.TestControlLoop as TestControlLoopAnalyzer
from ServoProjectModules.CalibrationAnalyzers import Helper
from ServoProjectModules import ArduinoManager

def openCreateConfigDialog(parent, configs):
    # pylint: disable=too-many-locals, too-many-statements
    dialog = Gtk.MessageDialog(
            transient_for=parent,
            flags=0,
            message_type=Gtk.MessageType.OTHER,
            buttons=Gtk.ButtonsType.OK_CANCEL,
            text='Create new configuration',
    )
    dialog.format_secondary_text(
        ''
    )
    box = dialog.get_message_area()
    createButton = dialog.get_widget_for_response(Gtk.ResponseType.OK)
    createButton.set_label('Create')
    createButton.set_sensitive(False)

    def onNewConfiNameEntryChange(widget):
        newName = widget.get_text()
        newFilePath = parent.arduinoSketchPath + '/config/' + newName

        def isValidConfigName(name):
            out = len(name) > 2
            out = out and name.find('.') == len(name) - 2
            out = out and name[-1] == 'h'
            out = out and name.find('/') == -1
            out = out and name.find('\\') == -1
            return out

        if (not os.path.isdir(newFilePath) and
            isValidConfigName(newName)):

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
    templateConfigCombo = (GuiFunctions.addTopLabelTo('<b>From template</b>', templateConfigCombo[0]),
                            templateConfigCombo[1])
    box1.pack_start(templateConfigCombo[0], False, False, 0)
    newNodeNrSpinButton = GuiFunctions.creatSpinButton(1, 1, 255, 1, getLowLev=True)
    newNodeNrSpinButton = (GuiFunctions.addTopLabelTo('<b>With new node number</b>', newNodeNrSpinButton[0]),
                            newNodeNrSpinButton[1])
    box1.pack_start(newNodeNrSpinButton[0], False, False, 0)
    box.pack_start(box1, False, False, 0)

    gearingEntry = GuiFunctions.createEntry('', getLowLev=True)
    gearingEntry = GuiFunctions.addTopLabelTo('<b>Gear ratio</b>\n'
        ' For best result calculate ratio by counting the gear teeth\n'
        '  Ex: 10 / 1 * 11 / 62 * 14 / 48 * 13 / 45 * 1 / 42', gearingEntry[0]), gearingEntry[1]
    box.pack_start(gearingEntry[0], False, False, 0)

    potentiometerRangeSpinButton = GuiFunctions.creatSpinButton(220, -360, 360, 1, getLowLev=True)

    encoderTypes = ['Potentiometer', 'Magnetic']

    def onEncoderTypeComboChange(widget):
        if widget.get_active() == 0:
            potentiometerRangeSpinButton[0].show_all()
        else:
            potentiometerRangeSpinButton[0].hide()

    box2 = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
    encoderTypeCombo = GuiFunctions.creatComboBox(encoderTypes[0], encoderTypes,
                                                    onChangeFun=onEncoderTypeComboChange, getLowLev=True)
    encoderTypeCombo = (GuiFunctions.addTopLabelTo('<b>Output encoder type</b>', encoderTypeCombo[0]),
                        encoderTypeCombo[1])
    box2.pack_start(encoderTypeCombo[0], False, False, 0)
    potentiometerRangeSpinButton = (GuiFunctions.addTopLabelTo('<b>Potentiometer range</b>',
                                        potentiometerRangeSpinButton[0]),
                                    potentiometerRangeSpinButton[1])
    box2.pack_start(potentiometerRangeSpinButton[0], False, False, 0)
    box.pack_start(box2, False, False, 0)

    newConfigurationNameEntry = GuiFunctions.createEntry('', onEdit=onNewConfiNameEntryChange, getLowLev=True)
    newConfigurationNameEntry = (GuiFunctions.addTopLabelTo('<b>Enter new name</b>\n (*.h)',
                                        newConfigurationNameEntry[0]),
                                    newConfigurationNameEntry[1])
    box.pack_start(newConfigurationNameEntry[0], False, False, 0)

    box.show_all()

    def getTemplateConfig():
        templateStr = templateConfigCombo[1].get_model()[templateConfigCombo[1].get_active()][0]
        descStartIndex = templateStr.find(' : ')
        if descStartIndex != -1:
            return templateStr[0: descStartIndex], templateStr[descStartIndex + 3:]
        return '', ''

    def getTemplateConfigClassString(configName, configClassName):
        configFilePath = parent.arduinoSketchPath + "/config/" + configName
        configFileAsString = ''

        with open(configFilePath, 'r', encoding='utf-8') as configFile:
            configFileAsString = configFile.read()

        configClassString = Helper.getConfigClassString(configFileAsString, configClassName)

        return configClassString

    def onTemplateConfigComboChange(widget):
        templateConfig = getTemplateConfig()
        configName = templateConfig[0]
        configClassName = templateConfig[1]
        configClassString = getTemplateConfigClassString(configName, configClassName)

        def hideEncoderSettings():
            gearingEntry[0].hide()
            encoderTypeCombo[0].hide()
            potentiometerRangeSpinButton[0].hide()

        if Helper.isSimulationConfig(configClassString):
            hideEncoderSettings()
        else:
            try:
                gearingStr = Helper.getConfiguredGearRatio(configClassString)
                gearingEntry[1].set_text(gearingStr)

                magneticEncoder, unitsPerRev = Helper.getConfiguredOutputEncoderData(configClassString)
                encoderTypeCombo[1].set_active(1 if magneticEncoder else 0)
                potentiometerRangeSpinButton[1].set_value(round(unitsPerRev / 4096 * 360))

                gearingEntry[0].show()
                encoderTypeCombo[0].show()
                potentiometerRangeSpinButton[0].show()
            except Exception:
                hideEncoderSettings()

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

        newConfigFileAsString = Helper.newConfigFileAsString(configClassString, newNodeNrSpinButton[1].get_value(),
                                                                configClassName)
        if newConfigFileAsString != '':
            configName = newConfigurationNameEntry[1].get_text()
            newConfigFilePath = parent.arduinoSketchPath + '/config/' + configName
            with open(newConfigFilePath, 'w', encoding='utf-8') as configFile:
                configFile.write(newConfigFileAsString)
                createdConfigName = configName

    dialog.destroy()

    return createdConfigName

class GuiWindow(Gtk.Window):
    def __init__(self, arduinoSketchPath):
        # pylint: disable=too-many-locals, too-many-statements
        self.startTime = time.time()

        Gtk.Window.__init__(self, title='Servo configuration', default_height=900, default_width=800)

        self.arduinoSketchPath = arduinoSketchPath
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
            with open('../.thankyou', 'w', encoding='utf-8') as f:
                f.write('Thankyou for staring the project!')

        def considerStaringMessage():
            if time.time() - self.startTime < 20.0:
                return

            try:
                with open('../.thankyou', 'r', encoding='utf-8') as f:
                    data = f.read()
                    if data == 'Thankyou for staring the project!':
                        return
            except Exception:
                pass

            dialog = Gtk.MessageDialog(
                    transient_for=self,
                    flags=0,
                    message_type=Gtk.MessageType.INFO,
                    buttons=Gtk.ButtonsType.YES_NO,
                    text='Consider staring',
            )
            dialog.format_secondary_text(
                'Has this project been useful for you?\nConsider giving it a \N{glowing star} on github'
            )
            response = dialog.run()
            dialog.destroy()
            if response == Gtk.ResponseType.YES:
                gotoToStaring()

        def starButtonClicked(widget):
            gotoToStaring()

        starButton = Gtk.Button(label=' \N{glowing star} Like')
        starButton.connect('clicked', starButtonClicked)
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
            configs = [c for c in configs if c not in (
                    'default.h',
                    'defaultSim.h',
                    'defaultDS3225.h',
                    'defaultMG90S.h')]

            currentItem = GuiFunctions.getActiveComboBoxItem(activeConfigCombo[1])
            GuiFunctions.setComboBoxItems(activeConfigCombo[1], currentItem, configs)

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
                _, configClassNames = self.getNodeNrAndClassNames(c)
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

            configFilePath = self.arduinoSketchPath + '/config/' + configName
            os.remove(configFilePath)
            selectActiveConfig('')
            loadConfigs(None, None)

        box2 = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
        box2.add(activeConfigCombo[0])
        activeConfigCombo = box2, activeConfigCombo[1]

        creatNewConfigButton = GuiFunctions.createButton('Create new', onClickFun=onCreateConfigClicked, width=10,
                                                            getLowLev=True)
        creatNewConfigButton[1].set_margin_start(0)
        creatNewConfigButton[1].set_margin_end(10)
        creatNewConfigButton[1].set_margin_top(10)
        creatNewConfigButton[1].set_margin_bottom(8)
        box2.add(creatNewConfigButton[0])

        deleteConfigButton = GuiFunctions.createButton('Delete', onClickFun=onDeleteConfigClicked, width=10,
                                                        getLowLev=True)
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

            for port, desc, _ in serial.tools.list_ports.comports():
                ports.append(str(port) + ': ' + str(desc))
            ports.sort()
            ports.append(': Simulation')

            currentItem = GuiFunctions.getActiveComboBoxItem(activeComPortCombo[1])
            if currentItem == '':
                currentItem = ports[0]

            ignoreComPortComboChangeEvent = True
            GuiFunctions.setComboBoxItems(activeComPortCombo[1], currentItem, ports)
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

        activeConfigCombo = (GuiFunctions.addTopLabelTo('<b>Configuration</b>', activeConfigCombo[0]),
                                activeConfigCombo[1])
        box0.pack_start(activeConfigCombo[0], False, False, 0)

        activeComPortCombo = (GuiFunctions.addTopLabelTo('<b>COM port</b>', activeComPortCombo[0]),
                                activeComPortCombo[1])
        box0.pack_start(activeComPortCombo[0], False, False, 0)

        activeNodeNrCombo = (GuiFunctions.addTopLabelTo('<b>Select node number</b>', activeNodeNrCombo[0]),
                                activeNodeNrCombo[1])

        calibrationCombo = None

        def onNodeNrChange(widget):
            if calibrationCombo and not ignoreComPortComboChangeEvent:
                calibrationCombo[1].set_active(0)  # pylint: disable=unsubscriptable-object

        activeNodeNrCombo[1].connect('changed', onNodeNrChange)

        def getComPortFromCombo():
            portString = activeComPortCombo[1].get_model()[activeComPortCombo[1].get_active()][0]
            portString = GuiFunctions.getActiveComboBoxItem(activeComPortCombo[1])
            descStartIndex = portString.find(':')
            if descStartIndex != -1:
                return portString[0: descStartIndex]
            return ''

        def onTranferToTarget(widget):
            transferThread = None

            dialog = Gtk.MessageDialog(
                    transient_for=self,
                    flags=0,
                    message_type=Gtk.MessageType.INFO,
                    buttons=Gtk.ButtonsType.NONE,
                    text='Transferring to target...',
            )
            dialog.format_secondary_text('')
            dialog.connect('delete-event', lambda w, e : transferThread.join())

            def resetAfterTransfer(ok):
                transferThread.join()
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

            transferThread = threading.Thread(target=transferThreadRun)
            transferThread.start()

            dialog.run()

        transferToTargetButton[1].connect('clicked', onTranferToTarget)

        lastActiveConfig = ''
        calibrationBox = None
        def onActiveConfigChange(widget):
            # pylint: disable=too-many-statements
            nonlocal lastActiveConfig
            nonlocal calibrationCombo

            activeIter = widget.get_active_iter()
            if activeIter is not None:
                model = widget.get_model()
                configName = model[activeIter][0]


                if configName == lastActiveConfig:
                    return

                lastActiveConfig = configName

                if calibrationCombo is not None:
                    box1.remove(calibrationCombo[0])  # pylint: disable=unsubscriptable-object
                    if calibrationBox is not None:
                        box1.remove(calibrationBox)

                if configName == '':
                    deleteConfigButton[1].set_sensitive(False)
                    transferToTargetButton[1].set_sensitive(False)
                    GuiFunctions.setComboBoxItems(activeNodeNrCombo[1], '', [''])
                    return

                if configName == 'usbToSerial.h':
                    deleteConfigButton[1].set_sensitive(False)
                else:
                    deleteConfigButton[1].set_sensitive(True)

                transferToTargetButton[1].set_sensitive(True)

                self.setActiveConfig(configName)

                nodeNrList, configClassNames = self.getNodeNrAndClassNames(configName)

                if len(nodeNrList) == 0 or len(configClassNames) == 0:
                    box1.remove(activeNodeNrCombo[0])
                    supportedCalibrationOptions = ['']

                else:
                    GuiFunctions.setComboBoxItems(activeNodeNrCombo[1], nodeNrList[0], nodeNrList)

                    if len(nodeNrList) == 1:
                        box1.remove(activeNodeNrCombo[0])
                    else:
                        box1.remove(activeNodeNrCombo[0])
                        box1.pack_start(activeNodeNrCombo[0], False, False, 0)

                    supportedCalibrationOptions = ['',
                            'Optical encoder',
                            'Pwm and system identification',
                            'Motor cogging torque',
                            'Output encoder calibration',
                            'Test control loop',
                            'Pwm nonlinearity (legacy)']

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

                    if calibrationBox is not None:
                        box1.remove(calibrationBox)

                    configFilePath = self.arduinoSketchPath + '/config/' + configName

                    if calibrationType == '':
                        calibrationBox = None

                    elif calibrationType == 'Pwm nonlinearity (legacy)':
                        nodeNr = getNodeNrFromCombo(activeNodeNrCombo[1])
                        getPortFun = getComPortFromCombo
                        configClassName = getConfigClassNameFromCombo(activeNodeNrCombo[1])
                        calibrationBox = PwmNonlinearityAnalyzer.createGuiBox(self, nodeNr, getPortFun,
                                                                                configFilePath, configClassName)

                    elif calibrationType == 'Optical encoder':
                        nodeNr = getNodeNrFromCombo(activeNodeNrCombo[1])
                        getPortFun = getComPortFromCombo
                        configClassName = getConfigClassNameFromCombo(activeNodeNrCombo[1])
                        calibrationBox = OpticalEncoderAnalyzer.createGuiBox(self, nodeNr, getPortFun,
                                                                                configFilePath, configClassName)

                    elif calibrationType == 'Pwm and system identification':
                        nodeNr = getNodeNrFromCombo(activeNodeNrCombo[1])
                        getPortFun = getComPortFromCombo
                        configClassName = getConfigClassNameFromCombo(activeNodeNrCombo[1])
                        calibrationBox = SystemIdentificationAnalyzer.createGuiBox(self, nodeNr, getPortFun,
                                                                                    configFilePath, configClassName)

                    elif calibrationType == 'Motor cogging torque':
                        nodeNr = getNodeNrFromCombo(activeNodeNrCombo[1])
                        getPortFun = getComPortFromCombo
                        configClassName = getConfigClassNameFromCombo(activeNodeNrCombo[1])
                        calibrationBox = MotorCoggingTorqueAnalyzer.createGuiBox(self, nodeNr, getPortFun,
                                                                                    configFilePath, configClassName)

                    elif calibrationType == 'Output encoder calibration':
                        nodeNr = getNodeNrFromCombo(activeNodeNrCombo[1])
                        getPortFun = getComPortFromCombo
                        configClassName = getConfigClassNameFromCombo(activeNodeNrCombo[1])
                        calibrationBox = OutputEncoderAnalyzer.createGuiBox(self, nodeNr, getPortFun,
                                                                            configFilePath, configClassName)

                    elif calibrationType in ('Test control loop'):
                        nodeNr = getNodeNrFromCombo(activeNodeNrCombo[1])
                        getPortFun = getComPortFromCombo
                        configClassName = getConfigClassNameFromCombo(activeNodeNrCombo[1])
                        calibrationBox = TestControlLoopAnalyzer.createGuiBox(self, nodeNr, getPortFun,
                                                                        configFilePath, configClassName)

                    if calibrationBox is not None:
                        box1.pack_start(calibrationBox, False, False, 0)

                calibrationCombo = GuiFunctions.creatComboBox('', supportedCalibrationOptions, onCaribrationTypeChange,
                                                                getLowLev=True)
                calibrationCombo = (GuiFunctions.addTopLabelTo('<b>Supported calibrations</b>', calibrationCombo[0]),
                                    calibrationCombo[1])
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
        with open(self.arduinoSketchPath + '/config/config.h', 'w', encoding='utf-8') as configFile:
            configFile.write('#include \"' + configName + '\"\n')

    def getNodeNrAndClassNames(self, configName):
        nodeNrList = []
        configClassNames = []

        configFilePath = self.arduinoSketchPath + '/config/' + configName
        configFileAsString = ''

        with open(configFilePath, 'r', encoding='utf-8') as configFile:
            configFileAsString = configFile.read()

            tempConfigFileAsString = configFileAsString
            dcServoPattern = re.compile(
                r'\n[^/]*make_unique\s*<\s*DCServoCommunicationHandler\w*\s*>\s*\((?P<nodeNr>[0-9]*)\s*,\s*'
                r'createDCServo\s*<\s*(?P<configClassName>\w+)\s*>')
            temp = dcServoPattern.search(tempConfigFileAsString)
            while temp is not None:
                nodeNrList.append(temp.group('nodeNr'))
                configClassNames.append(temp.group('configClassName'))
                tempConfigFileAsString = tempConfigFileAsString[temp.end(1):]
                temp = dcServoPattern.search(tempConfigFileAsString)

        return nodeNrList, configClassNames

    def setFocusOnTranferButton(self):
        self.transferToTargetButton.grab_focus()
