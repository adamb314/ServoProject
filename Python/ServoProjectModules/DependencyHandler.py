'''
Module for handling dependencies
'''

import os
import sys
import sysconfig
import re
import subprocess

def isMingwPlatform():
    return os.name == 'nt' and sysconfig.get_platform().startswith('mingw')

def checkPip():
    try:
        subprocess.check_output([sys.executable, '-m', 'pip'])
        return True
    except Exception:
        pass

    return False

def install(package):
    if isMingwPlatform():
        return installWithPacman(package)

    return installWithPip(package)

def installWithPip(package):
    try:
        subprocess.check_call([sys.executable, '-m', 'pip', 'install', package])
        return True
    except Exception:
        pass

    return False

pacmanPythonPackageName = None
def getPacmanPythonPackage():
    global pacmanPythonPackageName  # pylint: disable=global-statement
    if pacmanPythonPackageName:
        return pacmanPythonPackageName

    print('Looking for python package name with "pacman -Q"...')

    out = subprocess.check_output(['pacman', '-Q'])
    string = out.decode('utf-8')
    findPythonPattern = re.compile(r'(?P<pythonPackage>[^\s]+-python)\s')
    maches = findPythonPattern.finditer(string)

    for m in maches:
        pacmanPythonPackageName = m.group('pythonPackage')

    return pacmanPythonPackageName

def installWithPacman(package):
    try:
        getPacmanPythonPackage()
        package = f'{getPacmanPythonPackage()}-{package}'
        subprocess.check_call(['pacman', '-S', '--noconfirm', package])
        print('')
        return True
    except Exception:
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

def __init__(*, automaticInstall=False):
    lastMissingPackage = None
    while True:
        missingPackage = None
        try:
            # pylint: disable=import-outside-toplevel, unused-import
            from ServoProjectModules import ConfigurationGui

        except ModuleNotFoundError as e:
            missingPackage = e.name

        if not missingPackage:
            return True

        if missingPackage == lastMissingPackage:
            print('Please restart script to complete install')
            return False

        missingPackageData = getPackageData(missingPackage)

        if not isMingwPlatform() and not checkPip():
            print('Could not find python pip. Please install pip manually')
            return False

        if missingPackageData[1]:
            if not automaticInstall:
                ans = input(f'Missing dependency "{missingPackageData[0]}". Install? (y/N)?')
                if ans.find('Y') != 0 and ans.find('y') != 0:
                    return False

            if not install(missingPackageData[0]):
                print(f'Could not install "{missingPackageData[0]}". Please fix dependency manually!')
                return False

            lastMissingPackage = missingPackage
        else:
            print(f'Please fix missing dependency "{missingPackageData[0]}" manually!')
            return False
