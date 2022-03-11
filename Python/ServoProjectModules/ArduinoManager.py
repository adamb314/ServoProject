import requests
import platform
import re
import tarfile
import zipfile
import io
import os
import stat
import subprocess

arduinoCliFilename = ''

def runCommand(parameters, captureOutput=False):
    try:
        out = ''
        if captureOutput:
            out = subprocess.check_output(parameters)
            return True, out.decode("utf-8")
        else:
            subprocess.check_call(parameters)
            return True
    except Exception as e:
        pass

    if captureOutput:
        return False, ''

    return False

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
    findAssetPattern = re.compile(f'<a\\s+href\\s*=\\s*[^>]+releases/download/{versionInfo["tag_name"]}/(?P<asset>[^"]+)"')
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
    arduinoCliFilename = ''
    try:
        r = requests.get(url=url)
        if r.ok:
            arduinoCliBinData = None

            file_like_object = io.BytesIO(r.content)
            if url.find('.tar.gz') != -1:
                tar = tarfile.open(fileobj=file_like_object)
                arduinoCliObj = tar.extractfile('arduino-cli')
                arduinoCliBinData = arduinoCliObj.read()
                arduinoCliFilename = f'arduino-cli_{version}'
                
            elif url.find('.zip') != -1:
                zipObj = zipfile.ZipFile(file_like_object)
                arduinoCliBinData = zipObj.read('arduino-cli.exe')
                arduinoCliFilename = f'arduino-cli_{version}.exe'

            if path[-1] != '/':
                path += '/'
            arduinoCliFilename = f'{path}{arduinoCliFilename}'

            if arduinoCliBinData:
                with open(arduinoCliFilename, 'wb') as f:
                    f.write(arduinoCliBinData)

                    st = os.stat(arduinoCliFilename)
                    os.chmod(arduinoCliFilename, st.st_mode | stat.S_IEXEC)

                removeAllInOtherFiles(path, arduinoCliFilename)

    except Exception as e:
        arduinoCliFilename = ''

    return arduinoCliFilename

def __init__():
    global arduinoCliFilename
    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    tempBinPath = dname + '/tempBin'

    if not os.path.exists(tempBinPath):
        os.mkdir(tempBinPath)

    if runCommand(['arduino-cli', 'version'], captureOutput=True)[0]:
        arduinoCliFilename = 'arduino-cli'
        removeAllInOtherFiles(tempBinPath, '')
    else:
        arduinoCliVersionsList = [filename for filename in os.listdir(tempBinPath) if filename.find('arduino-cli') == 0]
        arduinoCliVersionsList.sort()

        if len(arduinoCliVersionsList) > 0:
            arduinoCliFilename = tempBinPath + '/' + arduinoCliVersionsList[-1]

        try:
            url, version = getLatestArduinoCliDownloadurl()

            temp = [filename for filename in arduinoCliVersionsList if filename.find(version) != -1]
            if len(temp) > 0:
                arduinoCliFilename = tempBinPath + '/' + temp[0]
            else:
                if arduinoCliFilename != '':
                    print(f'Found old arduino-cli ({os.path.basename(arduinoCliFilename)})')
                else:
                    print(f'Could not find arduino-cli')
                ans = input(f'Do you want to download latest version({version}) from GitHub? (y/N)')
                if ans.find('Y') == 0 or ans.find('y') == 0:
                    arduinoCliFilename = downloadArduinoCli(tempBinPath, url, version)
                    if arduinoCliFilename == '':
                        print(f'\nCould not download arduino-cli. Please install manually!')
        except Exception as e:
            pass

    if arduinoCliFilename != '':
        cores = [
            (r'adafruit:samd', 'adafruit:samd'),
        ]
        ok, coreListStr = runCommand([arduinoCliFilename, 'core', 'list'], captureOutput=True)
        if ok:
            adafruitAdditionalUrl = 'https://adafruit.github.io/arduino-board-index/package_adafruit_index.json'
            indexUpdated = False
            for core in cores:
                if not re.search(core[0], coreListStr):
                    ans = input(f'{core[1]} arduino core is missing... Do you want to install it? (y/N)')
                    if ans.find('Y') != 0 and ans.find('y') != 0:
                        continue
                    try:
                        if not indexUpdated:
                            subprocess.check_call([arduinoCliFilename, 'core', 'update-index', '--additional-urls', adafruitAdditionalUrl])
                            indexUpdated = True
                        subprocess.check_call([arduinoCliFilename, 'core', 'install', core[1], '--additional-urls', adafruitAdditionalUrl])
                    except Exception as e:
                        print(f'\nCould not install {core[1]}. Please install manually!')

        libraries = [
            (r'Adafruit.DotStar', 'Adafruit DotStar'),
            (r'Eigen', 'Eigen'),
        ]
        ok, libListStr = runCommand([arduinoCliFilename, 'lib', 'list'], captureOutput=True)
        if ok:
            for lib in libraries:
                if not re.search(lib[0], libListStr):
                    ans = input(f'The Arduino library "{lib[1]}" is missing... Do you want to install it? (y/N)')
                    if ans.find('Y') != 0 and ans.find('y') != 0:
                        continue
                    try:
                        subprocess.check_call([arduinoCliFilename, 'lib', 'install', lib[1]])
                    except Exception as e:
                        print(f'\nCould not install {lib[1]}. Please install manually!')


