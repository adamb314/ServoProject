#!/bin/python3
import sys
sys.path.insert(1, '../Python')
import ServoProjectModules.ConfigurationGui as ConfigurationGui
from ServoProjectModules.ConfigurationGui import Gtk

if __name__ == "__main__":
    window = ConfigurationGui.GuiWindow('.')
    window.connect("destroy", Gtk.main_quit)
    window.show_all()

    #work around for mathplotlib
    #calling Gtk.main_quit when all shown
    #figures are closed
    while not window.isClosed:
        Gtk.main()
