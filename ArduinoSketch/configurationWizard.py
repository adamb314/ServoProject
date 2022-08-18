#!/bin/python3

'''
Python script for managing Arduino configurations
'''

import os
import sys
import sysconfig
import re
import subprocess

def setWorkingDirToScriptDir():
    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    os.chdir(dname)

    sys.path.insert(1, '../Python')

def main():
    setWorkingDirToScriptDir()

    # pylint: disable=import-outside-toplevel
    from ServoProjectModules import DependencyHandler
    DependencyHandler.__init__(automaticInstall=False)

    from ServoProjectModules import ArduinoManager
    ArduinoManager.__init__(automaticInstall=False)

    from ServoProjectModules import ConfigurationGui
    from ServoProjectModules.ConfigurationGui import Gtk

    if not Gtk.init_check()[0]:
        print('\nGtk windowing system has not been initialized...\n'
            'Please run script in Gtk enabled environment!')
        return

    window = ConfigurationGui.GuiWindow('.')
    window.connect('destroy', Gtk.main_quit)
    window.show_all()

    #work around for mathplotlib
    #calling Gtk.main_quit when all shown
    #figures are closed
    while not window.isClosed:
        Gtk.main()

if __name__ == '__main__':
    main()
