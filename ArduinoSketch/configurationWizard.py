#!/bin/python3
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

def isMingwPlatform():
    return os.name == "nt" and sysconfig.get_platform().startswith("mingw")

def checkPip():
    try:
        out = subprocess.check_output([sys.executable, "-m", "pip"])
        return True
    except Exception as e:
        pass

    return False

def install(package):
    if isMingwPlatform():
        return installWithPacman(package)

    return installWithPip(package)

def installWithPip(package):
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", package])
        return True
    except Exception as e:
        pass

    return False

pacmanPythonPackageName = None
def getPacmanPythonPackage():
    global pacmanPythonPackageName
    if pacmanPythonPackageName:
        return pacmanPythonPackageName

    print('Looking for python package name with "pacman -Q"...')

    out = subprocess.check_output(["pacman", "-Q"])
    string = out.decode("utf-8")
    findPythonPattern = re.compile(r'(?P<pythonPackage>[^\s]+-python)\s')
    maches = findPythonPattern.finditer(string)

    for m in maches:
        pacmanPythonPackageName = m.group("pythonPackage")

    return pacmanPythonPackageName

def installWithPacman(package):
    try:
        getPacmanPythonPackage()
        package = f'{getPacmanPythonPackage()}-{package}'
        subprocess.check_call(["pacman", "-S", "--noconfirm", package])
        print('')
        return True
    except Exception as e:
        pass

    return False

def getPackageData(missingPackage):
    dic = {
        'serial' : ('pyserial', True),
        'gi' : ('pygobject', False),
    }

    if missingPackage in dic:
        return dic[missingPackage]

    return missingPackage, True

def main():
    setWorkingDirToScriptDir()

    lastMissingPackage = None
    while True:
        missingPackage = None
        try:
            import gi
            import ServoProjectModules.ConfigurationGui as ConfigurationGui
            from ServoProjectModules.ConfigurationGui import Gtk

            if Gtk.init_check()[0] == False:
                print(f'\nGtk windowing system has not been initialized...\nPlease run script in Gtk enabled environment!')
                break

            window = ConfigurationGui.GuiWindow('.')
            window.connect("destroy", Gtk.main_quit)
            window.show_all()

            #work around for mathplotlib
            #calling Gtk.main_quit when all shown
            #figures are closed
            while not window.isClosed:
                Gtk.main()

        except ModuleNotFoundError as e:
            missingPackage = e.name

        if not missingPackage:
            break

        if missingPackage == lastMissingPackage:
            print(f'Please restart script to complete install')
            break

        missingPackageData = getPackageData(missingPackage)

        if not isMingwPlatform() and not checkPip():
            print(f'Could not find python pip. Please install pip manually')
            break

        if missingPackageData[1]:
            ans = input(f'Missing dependency "{missingPackageData[0]}". Install? (y/N)?')
            if ans.find('Y') != 0 and ans.find('y') != 0:
                break

            if not install(missingPackageData[0]):
                print(f'Could not install "{missingPackageData[0]}". Please fix dependency manually!')
                break

            lastMissingPackage = missingPackage
        else:
            print(f'Please fix missing dependency "{missingPackageData[0]}" manually!')
            break

if __name__ == '__main__':
    main()
