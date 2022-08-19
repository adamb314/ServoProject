#!/usr/bin/env python3

'''
Python script for selecting active Arduino configuration
'''

# allow duplicate code since this script should be stand alone
# pylint: disable=duplicate-code

import os
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk  # pylint: disable=wrong-import-position

class ConfigSelectWindow(Gtk.Window):
    def __init__(self):
        Gtk.Window.__init__(self, title='Config Selector', default_height=50, default_width=300)

        self.vboxMain = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        self.add(self.vboxMain)

        currentConfigName = self.getSelectedConfigurationName()

        configurationNames = self.getConfigurations()

        activeIndex = -1

        self.configurations = Gtk.ListStore(str)
        for i, name in enumerate(configurationNames):
            n = []
            n.append(name)
            self.configurations.append(n)

            if currentConfigName == name:
                activeIndex = i

        self.updatePeriodComboBox = Gtk.ComboBox.new_with_model(self.configurations)
        self.updatePeriodComboBox.set_margin_start(5)
        self.updatePeriodComboBox.set_margin_end(5)
        self.updatePeriodComboBox.set_margin_top(10)
        self.updatePeriodComboBox.set_margin_bottom(8)
        rendererText = Gtk.CellRendererText()
        self.updatePeriodComboBox.pack_start(rendererText, True)
        self.updatePeriodComboBox.add_attribute(rendererText, 'text', 0)
        self.updatePeriodComboBox.set_active(activeIndex)
        self.updatePeriodComboBox.connect('changed', self.onConfigSelected)

        self.vboxMain.pack_start(self.updatePeriodComboBox, False, False, 0)

        self.closeButton = Gtk.Button(label='Close')
        self.closeButton.connect('clicked', self.onCloseButtonPressed)
        self.closeButton.set_margin_start(5)
        self.closeButton.set_margin_end(5)
        self.closeButton.set_margin_top(8)
        self.closeButton.set_margin_bottom(10)
        self.closeButton.set_property('width-request', 120)
        self.vboxMain.pack_end(self.closeButton, False, False, 0)

        self.closeButton.grab_focus()

    def onConfigSelected(self, widget):
        activeIter = widget.get_active_iter()
        if activeIter is not None:
            model = widget.get_model()
            configName = model[activeIter][0]

            with open('config/config.h', 'w', encoding='utf-8') as configFile:
                configFile.write('#include \"' + configName + '\"\n')

    def getConfigurations(self):
        configs = []
        path = 'config'
        for filename in os.listdir(path):
            ipath = path + '/' + filename
            if not os.path.isdir(ipath) and not filename == 'config.h' and filename[-2:] == '.h':
                configs.append(filename)

        configs.sort()
        return configs

    def getSelectedConfigurationName(self):
        if os.path.exists('config/config.h'):
            configFileAsString = ''
            with open('config/config.h', 'r', encoding='utf-8') as configFile:
                configFileAsString = configFile.read()
            startStr = '#include \"'
            if configFileAsString.find(startStr) == 0:
                endStr = '\"'
                configFileAsString = configFileAsString[len(startStr):]
                endI = configFileAsString.find(endStr)
                if endI != -1:
                    return configFileAsString[0:endI]
        return ''

    def onCloseButtonPressed(self, widget):
        Gtk.main_quit()

if __name__ == '__main__':
    window = ConfigSelectWindow()
    window.connect('destroy', Gtk.main_quit)
    window.show_all()

    Gtk.main()
