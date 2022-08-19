'''
Module for handling ArduinoCli functionality
'''

import platform
import re
import tarfile
import zipfile
import io
import os
import stat
import subprocess
import requests

def runCommand(parameters, captureOutput=False):
    try:
        out = ''
        if captureOutput:
            out = subprocess.check_output(parameters)
            return True, out.decode("utf-8")

        subprocess.check_call(parameters)
        return True
    except Exception:
        pass

    if captureOutput:
        return False, ''

    return False

arduinoCliFilename = ''

def transfer(port):
    if arduinoCliFilename == '':
        print('Cannot transfer without arduino-cli!')
        return False

    print('Compiling Arduino Sketch...')
    ok = runCommand([arduinoCliFilename, 'compile', '--fqbn', 'adafruit:samd:adafruit_itsybitsy_m0'])
    if not ok:
        return False

    if port == '':
        return True

    print(f'Transferring Arduino Sketch to {port}...')
    return runCommand([arduinoCliFilename, 'upload', f'-p{port}', '--fqbn', 'adafruit:samd:adafruit_itsybitsy_m0'])

def getListOfLatestGitHubReleasAssets(url):
    if url[-1] != '/':
        url = url + '/'

    print('Looking up latest version of "arduino-cli"...')
    r = requests.get(url=f'{url}releases/latest', headers={'Accept': 'application/json'}, allow_redirects=True)

    versionInfo = r.json()
    r = requests.get(url=f'{url}releases/tag/{versionInfo["tag_name"]}', allow_redirects=True)
    if not r.ok:
        return [], ''

    releaseHtmlStr = str(r.content)
    findAssetPattern = re.compile(
            f'<a\\s+href\\s*=\\s*[^>]+releases/download/{versionInfo["tag_name"]}/(?P<asset>[^"]+)"')
    maches = findAssetPattern.finditer(releaseHtmlStr)

    out =[]
    for m in maches:
        out.append(f'{url}releases/download/{versionInfo["tag_name"]}/{m.group("asset")}'.lower())

    return out, versionInfo["tag_name"]

def getArduinoCliPlatformNaming():
    return f'{platform.system()}_{platform.architecture()[0]}'.lower()

def getLatestArduinoCliDownloadurl():
    platformStr = getArduinoCliPlatformNaming()

    l, version = getListOfLatestGitHubReleasAssets('http://github.com/arduino/arduino-cli')
    l = [s for s in l if s.find('.txt') == -1]
    l = [s for s in l if s.find(platformStr) != -1]
    return l[0], version

def removeAllInOtherFiles(path, keepFilename):
    for name in os.listdir(path):
        if path[-1] != '/':
            path += '/'
        filename = f'{path}{name}'
        if filename != keepFilename:
            os.remove(filename)

def downloadArduinoCli(path, url, version):
    filename = ''
    try:
        r = requests.get(url=url)
        if r.ok:
            arduinoCliBinData = None

            fileLikeObject = io.BytesIO(r.content)
            if url.find('.tar.gz') != -1:
                with tarfile.open(fileobj=fileLikeObject) as tar:
                    arduinoCliObj = tar.extractfile('arduino-cli')
                    arduinoCliBinData = arduinoCliObj.read()
                    filename = f'arduino-cli_{version}'

            elif url.find('.zip') != -1:
                with zipfile.ZipFile(fileLikeObject) as zipObj:
                    arduinoCliBinData = zipObj.read('arduino-cli.exe')
                    filename = f'arduino-cli_{version}.exe'

            if path[-1] != '/':
                path += '/'
            filename = f'{path}{filename}'

            if arduinoCliBinData:
                with open(filename, 'wb') as f:
                    f.write(arduinoCliBinData)

                    st = os.stat(filename)
                    os.chmod(filename, st.st_mode | stat.S_IEXEC)

                removeAllInOtherFiles(path, filename)

    except Exception:  # pylint: disable=broad-except
        filename = ''

    return filename

def handleArduinoCliDependency(automaticInstall=False):
    activeArduinoCli = ''

    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    tempBinPath = dname + '/tempBin'

    if not os.path.exists(tempBinPath):
        os.mkdir(tempBinPath)

    if runCommand(['arduino-cli', 'version'], captureOutput=True)[0]:
        activeArduinoCli = 'arduino-cli'
        removeAllInOtherFiles(tempBinPath, '')
        return activeArduinoCli

    arduinoCliVersionsList = [filename for filename in os.listdir(tempBinPath) if filename.find('arduino-cli') == 0]
    arduinoCliVersionsList.sort()

    if len(arduinoCliVersionsList) > 0:
        activeArduinoCli = tempBinPath + '/' + arduinoCliVersionsList[-1]

    try:
        url, version = getLatestArduinoCliDownloadurl()

        temp = [filename for filename in arduinoCliVersionsList if filename.find(version) != -1]
        if len(temp) > 0:
            activeArduinoCli = tempBinPath + '/' + temp[0]
        else:
            if activeArduinoCli != '':
                print(f'Found old arduino-cli ({os.path.basename(activeArduinoCli)})')
            else:
                print('Could not find arduino-cli')
            if not automaticInstall:
                ans = input(f'Do you want to download latest version({version}) from GitHub? (y/N)')
                if ans.find('Y') != 0 and ans.find('y') != 0:
                    return activeArduinoCli
            print('Downloading latest arduino-cli')
            activeArduinoCli = downloadArduinoCli(tempBinPath, url, version)
            if activeArduinoCli == '':
                print('\nCould not download arduino-cli. Please install manually!')
    except Exception:  # pylint: disable=broad-except
        pass

    return activeArduinoCli

def handleArduinoCoreDependencies(cores, automaticInstall=False):
    ok, coreListStr = runCommand([arduinoCliFilename, 'core', 'list'], captureOutput=True)
    if not ok:
        return

    adafruitAdditionalUrl = 'https://adafruit.github.io/arduino-board-index/package_adafruit_index.json'
    indexUpdated = False
    for core in cores:
        if re.search(core[0], coreListStr):
            continue

        if not automaticInstall:
            ans = input(f'{core[1]} arduino core is missing... Do you want to install it? (y/N)')
            if ans.find('Y') != 0 and ans.find('y') != 0:
                continue
        try:
            if not indexUpdated:
                subprocess.check_call([arduinoCliFilename, 'core',
                                        'update-index', '--additional-urls', adafruitAdditionalUrl])
                indexUpdated = True
            subprocess.check_call([arduinoCliFilename, 'core', 'install', core[1],
                                    '--additional-urls', adafruitAdditionalUrl])
        except Exception:  # pylint: disable=broad-except
            print(f'\nCould not install {core[1]}. Please install manually!')

def handleArduinoLibDependencies(libraries, automaticInstall=False):
    ok, libListStr = runCommand([arduinoCliFilename, 'lib', 'list'], captureOutput=True)
    if not ok:
        return

    for lib in libraries:
        if re.search(lib[0], libListStr):
            continue

        if not automaticInstall:
            ans = input(f'The Arduino library "{lib[1]}" is missing... Do you want to install it? (y/N)')
            if ans.find('Y') != 0 and ans.find('y') != 0:
                continue
        try:
            subprocess.check_call([arduinoCliFilename, 'lib', 'install', lib[1]])
        except Exception:  # pylint: disable=broad-except
            print(f'\nCould not install {lib[1]}. Please install manually!')

def __init__(automaticInstall=False):
    global arduinoCliFilename  # pylint: disable=global-statement

    arduinoCliFilename = handleArduinoCliDependency(automaticInstall)
    if arduinoCliFilename == '':
        return

    cores = [
        (r'adafruit:samd', 'adafruit:samd'),
    ]
    handleArduinoCoreDependencies(cores, automaticInstall)

    libraries = [
        (r'Adafruit.DotStar', 'Adafruit DotStar'),
        (r'Eigen', 'Eigen'),
    ]
    handleArduinoLibDependencies(libraries, automaticInstall)

if __name__ == '__main__':
    __init__(automaticInstall=True)
