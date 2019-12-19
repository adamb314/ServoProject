#!/usr/bin/env python3
import os
import gi
gi.require_version("Gtk", "3.0")
from gi.repository import GLib, Gtk

class ConfigSelectWindow(Gtk.Window):
    """docstring for ConfigSelectWindow"""
    def __init__(self):
        Gtk.Window.__init__(self, title="Config Selector", default_height=50, default_width=300)

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
        self.updatePeriodComboBox.set_margin_end(10)
        self.updatePeriodComboBox.set_margin_top(10)
        self.updatePeriodComboBox.set_margin_bottom(8)
        renderer_text = Gtk.CellRendererText()
        self.updatePeriodComboBox.pack_start(renderer_text, True)
        self.updatePeriodComboBox.add_attribute(renderer_text, "text", 0)
        self.updatePeriodComboBox.set_active(activeIndex)
        self.updatePeriodComboBox.connect("changed", self.onConfigSelected)

        self.vboxMain = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        self.add(self.vboxMain)
        self.vboxMain.pack_start(self.updatePeriodComboBox, False, False, 0)

    def onConfigSelected(self, widget):
        activeIter = widget.get_active_iter()
        if activeIter is not None:
            model = widget.get_model()
            configName = model[activeIter][0]

            configFile = open("config/config.h", "w")
            configFile.write("#include \"" + configName + "\"\n")
            configFile.close()

    def getConfigurations(self):
        configs = []
        path = "config"
        for filename in os.listdir(path):
            ipath = path + "/" + filename
            if not os.path.isdir(ipath) and not filename == "config.h":
                configs.append(filename)

        return configs

    def getSelectedConfigurationName(self):
        if os.path.exists("config/config.h"):
            configFile = open("config/config.h", "r")
            configName = configFile.read()
            configFile.close()
            startStr = "#include \""
            if configName.find(startStr) == 0:
                endStr = "\""
                configName = configName[len(startStr):]
                endI = configName.find(endStr)
                if not endI == -1:
                    return configName[0:endI]
        return ""

if __name__ == '__main__':
    window = ConfigSelectWindow()
    window.connect("destroy", Gtk.main_quit)
    window.show_all()

    Gtk.main()
