'''
Module for calibrating the optical encoder
'''
# pylint: disable=too-many-lines, duplicate-code

from ServoProjectModules.CalibrationAnalyzers.Helper import *  # pylint: disable=wildcard-import, unused-wildcard-import

@numba.jit(nopython=True)
def calcCovWithEndOfVectors(aVec, bVec, ca, cb, noiseDepresMemLenght):
    cov = 0

    aDiff = 0
    bDiff = 0

    for i, (a, b) in enumerate(zip(aVec[-noiseDepresMemLenght:], bVec[-noiseDepresMemLenght:])):
        a += aDiff * (noiseDepresMemLenght - i - 1)
        b += bDiff * (noiseDepresMemLenght - i - 1)

        cov += (ca - a)**2 + (cb - b)**2

    return cov

@numba.jit(nopython=True)
def findBestFitt(data, modData, aVec, bVec, noiseDepresMemLenght):
    minCov = 1000000
    minIndex = 0
    done = False
    wrapIndex = 0

    for i, d in enumerate(modData):
        cov = calcCovWithEndOfVectors(aVec, bVec, d[0], d[1], noiseDepresMemLenght)

        if minCov > cov:
            minIndex = i
            minCov = cov

    if len(modData) < len(data) * 0.2:
        for i in range(noiseDepresMemLenght, int(len(aVec) / 2)):
            cov = calcCovWithEndOfVectors(aVec, bVec, aVec[i], bVec[i], noiseDepresMemLenght)

            if minCov > cov:
                wrapIndex = i
                done = True
                break

    return (minIndex, done, wrapIndex)

@numba.jit(nopython=True)
def findBestFitt2(ca, cb, aVec, bVec):
    noiseDepresMemLenght = int(round(8 - 2))
    minCov = math.inf
    minIndex = 0

    for i, _ in enumerate(zip(aVec[1:], bVec[1:])):
        cov = 0
        n = 0
        iMin = i - noiseDepresMemLenght
        iMax = i + 2 + noiseDepresMemLenght
        if iMin < 0:
            temp = iMin + len(aVec)
            for (a, b) in zip(aVec[temp:], bVec[temp:]):
                cov += (ca - a)**2 + (cb - b)**2
                n += 1
            iMin = 0
        if iMax > len(aVec):
            temp = iMax - len(aVec)
            for (a, b) in zip(aVec[0:temp], bVec[0:temp]):
                cov += (ca - a)**2 + (cb - b)**2
                n += 1
            iMax = len(aVec)

        for (a, b) in zip(aVec[iMin:iMax], bVec[iMin:iMax]):
            cov += (ca - a)**2 + (cb - b)**2
            n += 1
        cov = cov / n

        if minCov > cov:
            minIndex = i
            minCov = cov

    return minIndex + 1

@numba.jit(nopython=True)
def calcFitCov(atIndex, a, b, aVec, bVec, noiseDepresMemLenght):
    # pylint: disable=too-many-arguments
    cov = 0
    iMin = atIndex - noiseDepresMemLenght
    iMax = atIndex + 2 + noiseDepresMemLenght
    for i in range(iMin, iMax):
        i = i % len(aVec)
        cov += (a - aVec[i])**2 + (b - bVec[i])**2

    return cov

@numba.jit(nopython=True)
def findBestFitt2Opt(a, b, aVec, bVec, noiseDepresMemLenght):
    noiseDepresMemLenght = int(round(noiseDepresMemLenght - 2))
    minCov = math.inf
    minIndex = 0

    startStep = int(math.sqrt(len(aVec)))
    for i in range(0, len(aVec), startStep):
        cov = calcFitCov(i, a, b, aVec, bVec, noiseDepresMemLenght)
        if minCov > cov:
            minIndex = i
            minCov = cov

    for i in range(minIndex - startStep, minIndex + startStep):
        i = i % len(aVec)
        cov = calcFitCov(i, a, b, aVec, bVec, noiseDepresMemLenght)
        if minCov > cov:
            minIndex = i
            minCov = cov

    return (minIndex + 1) % len(aVec)

@numba.jit(nopython=True)
def findWorstFitt(aVec, bVec):
    maxCov = 0
    maxIndex = 0

    for i, d in enumerate(zip(aVec[1:], bVec[1:])):
        a = d[0]
        b = d[1]

        nextI = i + 2
        if nextI == len(aVec):
            nextI = -1

        cov = (aVec[i] - a)**2 + 0 * (bVec[i] - b)**2 + (aVec[nextI] - a)**2 + 0 * (bVec[nextI] - b)**2

        if maxCov < cov:
            maxIndex = i
            maxCov = cov

    return maxIndex + 1

def calcTotalCost(aVec, bVec, refAVec, refBVec):
    cost = 0
    for d in zip(aVec, bVec, refAVec, refBVec):
        cost += (d[0] - d[2])**2 + (d[1] - d[3])**2
    return cost

def getShift(aVec, bVec, refAVec, refBVec):

    tempA = []
    tempB = []
    for d in zip(aVec, bVec):
        tempA.append(d[0])
        tempB.append(d[1])

    aVec = tempA
    bVec = tempB

    bestFittShift = 0
    bestFittCost = float('inf')
    for shift in range(0, len(aVec)):
        tempA = aVec[-shift:] + aVec[0:-shift]
        tempB = bVec[-shift:] + bVec[0:-shift]

        cost = calcTotalCost(tempA, tempB, refAVec, refBVec)

        if cost < bestFittCost:
            bestFittCost = cost
            bestFittShift = shift

    return bestFittShift

def shiftVec(vec, shift):
    temp = []
    for d in vec:
        temp.append(d)
    return temp[-shift:] + temp[0:-shift]

class OpticalEncoderDataVectorGenerator:
    # pylint: disable=too-many-instance-attributes
    _aVecPattern = re.compile(
            r'(?P<beg>.*createMainEncoderHandler\(\)\s*\{(?:.*\n)*?\s*(constexpr)?\s+'
            r'(static)?\s+std\s*::\s*array\s*<\s*uint16_t\s*,\s*)'
            r'\d+(?P<end>\s*>\s*aVec\s*=\s*)\{(?P<vec>[\d\s,]*)\};')
    _bVecPattern = re.compile(
            r'(?P<beg>.*createMainEncoderHandler\(\)\s*\{(?:.*\n)*?\s*(constexpr)?\s+'
            r'(static)?\s+std\s*::\s*array\s*<\s*uint16_t\s*,\s*)'
            r'\d+(?P<end>\s*>\s*bVec\s*=\s*)\{(?P<vec>[\d\s,]*)\};')

    def __init__(self, data, configFileAsString='', configClassName='', *, segment=3000,
            constVelIndex=10000, noiseDepresMemLenght=5,
            shouldAbort=None, updateProgress=None):
        # pylint: disable=too-many-locals, too-many-branches, too-many-statements

        self.data = data[:, 0:2]
        self.noiseDepresMemLenght = noiseDepresMemLenght
        self.constVelIndex = constVelIndex

        self.shouldAbort = shouldAbort
        self.updateProgress = updateProgress

        a0 = self.data[0, 0]
        b0 = self.data[0, 1]
        armed = 0
        startOfFirstRotation = None
        endOfFirstRotation = None
        meanCost = sum(((d[0] - a0)**2 + (d[1] -b0)**2 for d in self.data[0:constVelIndex, 0:2])) / constVelIndex

        for i, d in enumerate(self.data[0:constVelIndex, 0:2]):
            c = (d[0] - a0)**2 + (d[1] -b0)**2

            if startOfFirstRotation is None:
                if c > meanCost:
                    startOfFirstRotation = i
            elif endOfFirstRotation is None:
                if armed == 0:
                    if c > meanCost * 1.5:
                        armed = 1
                elif armed == 1:
                    if c < meanCost * 0.5:
                        armed = 2
                else:
                    if c > meanCost:
                        endOfFirstRotation = i
                        break

        if startOfFirstRotation is None or endOfFirstRotation is None:
            raise Exception('No movement detected')

        startIndex = startOfFirstRotation
        firstRotationLenght = endOfFirstRotation - startOfFirstRotation
        dirNoiseDepresMemLenght = int(math.ceil(firstRotationLenght / 10))

        self.a0 = self.data[startIndex, 0]
        self.b0 = self.data[startIndex, 1]
        self.a1 = self.data[startIndex + dirNoiseDepresMemLenght, 0]
        self.b1 = self.data[startIndex + dirNoiseDepresMemLenght, 1]

        self.aVecList = []
        self.bVecList = []

        aVecMean = np.zeros(segment)
        bVecMean = np.zeros(segment)

        def filterOutStationarySegments(data):
            filteredData = []
            filterSeg = []
            for d in data:
                if len(filterSeg) < 10:
                    filterSeg.append([d[0], d[1]])
                else:
                    cov = 0
                    s = filterSeg[0]
                    for e in filterSeg[1:]:
                        cov += (e[0] - s[0])**2
                        cov += (e[1] - s[1])**2

                    if cov > 10000:
                        for d in filterSeg:
                            filteredData.append(d)

                    filterSeg = []
            return filteredData

        filteredData = filterOutStationarySegments(data)
        filteredData = np.array(filteredData[constVelIndex:])
        nr = int(len(filteredData) / segment)

        actNr = 0
        for i in range(0, nr):
            if self.shouldAbort is not None:
                if self.shouldAbort():
                    return

            try:
                aVec, bVec = self.genVec(filteredData[segment * i: segment * (i + 1)], 1.0 / nr * 0.9, i / nr * 0.9)
                aVec = np.array(shrinkArray(aVec, 4096))
                bVec = np.array(shrinkArray(bVec, 4096))

                self.aVecList.append(aVec)
                self.bVecList.append(bVec)

                aVecMean += aVec
                bVecMean += bVec

                actNr += 1
                time.sleep(0.1)
            except Exception as e:
                print(format(e))

        if actNr == 0:
            raise Exception('Not enough data')

        aVecMean = aVecMean / actNr
        bVecMean = bVecMean / actNr

        def findBestVecIndex(aVecList, bVecList, refAVec, refBVec):
            minCost = float('inf')
            bestVecIndex = 0
            for i, d in enumerate(zip(aVecList, bVecList)):
                cost = calcTotalCost(d[0], d[1], refAVec, refBVec)

                if cost < minCost:
                    minCost = cost
                    bestVecIndex = i
            return bestVecIndex

        unsortedAVecList = self.aVecList[:]
        unsortedBVecList = self.bVecList[:]
        self.aVecList = []
        self.bVecList = []
        while len(unsortedAVecList) > 0:
            i = findBestVecIndex(unsortedAVecList, unsortedBVecList, aVecMean, bVecMean)
            self.aVecList.append(unsortedAVecList[i])
            self.bVecList.append(unsortedBVecList[i])
            del unsortedAVecList[i]
            del unsortedBVecList[i]

        mergedAVectors = numba.typed.List(self.aVecList[0])
        mergedBVectors = numba.typed.List(self.bVecList[0])
        inserts = []
        for d in mergedAVectors:
            inserts.append([])

        nrOfVectorsToMerge = len(self.aVecList[1:])
        for i, d in enumerate(zip(self.aVecList[1:], self.bVecList[1:])):
            tempAVec = numba.typed.List(d[0])
            tempBVec = numba.typed.List(d[1])

            initialTempVecLenght = len(tempAVec)
            while len(tempAVec) > 0:
                minIndex = findBestFitt2Opt(tempAVec[0], tempBVec[0], mergedAVectors, mergedBVectors,
                                            2 * i + self.noiseDepresMemLenght)
                inserts[minIndex].append((tempAVec[0], tempBVec[0]))
                del tempAVec[0]
                del tempBVec[0]

                if self.updateProgress is not None:
                    fraction = 1.0 - len(tempAVec) / initialTempVecLenght
                    fraction = (i + fraction) / nrOfVectorsToMerge
                    self.updateProgress(0.1 * fraction + 0.9)

            newMergedAVec = numba.typed.List()
            newMergedBVec = numba.typed.List()
            for d in zip(mergedAVectors, mergedBVectors, inserts):
                newMergedAVec.append(d[0])
                newMergedBVec.append(d[1])
                for d2 in d[2]:
                    newMergedAVec.append(d2[0])
                    newMergedBVec.append(d2[1])

            mergedAVectors = newMergedAVec
            mergedBVectors = newMergedBVec
            inserts = []
            for d in mergedAVectors:
                inserts.append([])

        self.mergedAVectors = mergedAVectors
        self.mergedBVectors = mergedBVectors
        self.aVec = np.array(shrinkArray(self.mergedAVectors, 2048))
        self.bVec = np.array(shrinkArray(self.mergedBVectors, 2048))

        self.oldAVec = None
        self.oldBVec = None

        configClassString = getConfigClassString(configFileAsString, configClassName)

        if configClassString != '':
            temp1 = OpticalEncoderDataVectorGenerator._aVecPattern.search(configClassString)
            temp2 = OpticalEncoderDataVectorGenerator._bVecPattern.search(configClassString)

            if temp1 is not None and temp2 is not None:
                oldAVecStr = temp1.group('vec')
                oldBVecStr = temp2.group('vec')
                oldAVecStr = '[' + oldAVecStr + ']'
                oldBVecStr = '[' + oldBVecStr + ']'

                self.oldAVec = eval(oldAVecStr)  # pylint: disable=eval-used
                self.oldBVec = eval(oldBVecStr)  # pylint: disable=eval-used

                if len(self.oldAVec) <= 1:
                    self.oldAVec = None
                if len(self.oldBVec) <= 1:
                    self.oldBVec = None

        if self.oldAVec is not None and self.oldBVec is not None:
            bestFittShift = getShift(self.aVec, self.bVec, self.oldAVec, self.oldBVec)
            self.aVecShifted = shiftVec(self.aVec, bestFittShift)
            self.bVecShifted = shiftVec(self.bVec, bestFittShift)
        else:
            self.aVecShifted = self.aVec
            self.bVecShifted = self.bVec

    def genVec(self, data, partOfTotal = 1.0, donePrev = 0.0):
        # pylint: disable=too-many-locals, too-many-branches, too-many-statements
        aVec = numba.typed.List()
        bVec = numba.typed.List()
        for _ in range(0, self.noiseDepresMemLenght):
            aVec.append(self.a0)
            bVec.append(self.b0)

        modData = data[:]
        wrapIndex = 0

        while len(modData) > 0:
            if self.shouldAbort is not None:
                if self.shouldAbort():
                    return (np.array(aVec), np.array(bVec))

            if self.updateProgress is not None:
                fraction = (len(data) - len(modData)) / len(data)
                self.updateProgress(partOfTotal * fraction + donePrev)

            minIndex, done, wrapIndex = findBestFitt(data, modData, aVec, bVec, self.noiseDepresMemLenght)

            if done:
                break

            aVec.append(modData[minIndex, 0])
            bVec.append(modData[minIndex, 1])

            modData = np.delete(modData, minIndex, 0)

        temp = []
        for d in modData:
            temp.append(d[0:2])
        for d in zip(aVec[self.noiseDepresMemLenght:wrapIndex], bVec[self.noiseDepresMemLenght:wrapIndex]):
            temp.append(np.array(d))
        modData = np.array(temp)
        aVec = aVec[wrapIndex:]
        bVec = bVec[wrapIndex:]

        while len(modData) > 0:
            if self.shouldAbort is not None:
                if self.shouldAbort():
                    return (np.array(aVec), np.array(bVec))

            if self.updateProgress is not None:
                fraction = (len(data) - len(modData)) / len(data)
                self.updateProgress(partOfTotal * fraction + donePrev)

            minIndex = findBestFitt2Opt(modData[0, 0], modData[0, 1], aVec, bVec, self.noiseDepresMemLenght)

            aVec.insert(minIndex, modData[0, 0])
            bVec.insert(minIndex, modData[0, 1])

            modData = np.delete(modData, 0, 0)

        dirNoiseDepresMemLenght = int(len(aVec) / 10)

        if abs(self.a1 - self.a0) < 2 and abs(self.b1 - self.b0) < 2:
            raise Exception('No movement detected 2')

        dirSign = 0
        if abs(self.a1 - self.a0) > abs(self.b1 - self.b0):
            dirSign = (aVec[dirNoiseDepresMemLenght] - aVec[0]) / (self.a1 - self.a0)
        else:
            dirSign = (bVec[dirNoiseDepresMemLenght] - bVec[0]) / (self.b1 - self.b0)

        if dirSign < 0:
            aVec = aVec[::-1]
            bVec = bVec[::-1]

        return (np.array(aVec), np.array(bVec))

    def showAdditionalDiagnosticPlots(self):
        # pylint: disable=too-many-locals, too-many-statements
        sensorValueOffset = 0
        predictNextPos = 0
        iAtLastOffsetUpdate = 0
        lastMinCostIndex = 0

        def calcCost(i, a, b, aVec, bVec):
            vecSize = len(aVec)

            if i >= vecSize:
                i -= vecSize
            elif i < 0:
                i += vecSize

            tempA = aVec[i] - a
            tempA = tempA * tempA

            tempB = bVec[i] - b
            tempB = tempB * tempB

            return (tempA + tempB, i)

        def calculatePosition(ca, cb, aVec, bVec):
            # pylint: disable=too-many-locals, too-many-statements
            nonlocal sensorValueOffset
            nonlocal predictNextPos
            nonlocal iAtLastOffsetUpdate
            nonlocal lastMinCostIndex

            sensor1Value = ca
            sensor2Value = cb
            vecSize = len(aVec)

            #int16_t offset = sensorValueOffset
            diagnosticDataA = sensor1Value
            diagnosticDataB = sensor2Value
            #sensor1Value -= offset
            #sensor2Value -= offset

            stepSize = int(vecSize / 2.0 + 1)

            i = predictNextPos
            cost, i = calcCost(i, sensor1Value, sensor2Value, aVec, bVec)

            bestI = i
            bestCost = cost

            i += stepSize
            while i < vecSize:
                cost, i = calcCost(i, sensor1Value, sensor2Value, aVec, bVec)

                if cost < bestCost:
                    bestI = i
                    bestCost = cost

                i += stepSize

            checkDir = 1

            while stepSize != 1:
                i = bestI

                if stepSize <= 4:
                    stepSize -= 1
                else:
                    stepSize = int(stepSize / 2)
                    if stepSize == 0:
                        stepSize = 1

                i += stepSize * checkDir

                cost, i = calcCost(i, sensor1Value, sensor2Value, aVec, bVec)

                if cost < bestCost:
                    bestI = i
                    bestCost = cost

                    checkDir *= -1

                    continue

                i -= 2 * stepSize * checkDir

                cost, i = calcCost(i, sensor1Value, sensor2Value, aVec, bVec)

                if cost < bestCost:
                    bestI = i
                    bestCost = cost

            bestI = int(bestI)

            if abs(bestI - iAtLastOffsetUpdate) > (vecSize / 10):
                iAtLastOffsetUpdate = bestI

                a = aVec[bestI]
                b = bVec[bestI]
                currentOffset = (diagnosticDataA - a + diagnosticDataB - b) / 2

                sensorValueOffset = 0.95 * sensorValueOffset + 0.05 * currentOffset

            lastMinCostIndexChange = bestI - lastMinCostIndex
            lastMinCostIndex = bestI
            predictNextPos = bestI + lastMinCostIndexChange
            while predictNextPos >= vecSize:
                predictNextPos -= vecSize
            while predictNextPos < 0:
                predictNextPos += vecSize

            return (bestI, bestCost)

        fig = plt.figure(1)
        fig.suptitle('Merged sensor characteristic vectors')
        plt.plot(self.mergedAVectors, 'r-')
        plt.plot(self.mergedBVectors, 'g-')

        fig = plt.figure(2)
        fig.suptitle('Partial sensor characteristic vectors')
        for d in zip(self.aVecList, self.bVecList):
            x = np.arange(len(d[0])) * len(self.aVec) / len(d[0])
            plt.plot(x, d[0])
            plt.plot(x, d[1])
        plt.plot(self.aVec, 'r-')
        plt.plot(self.bVec, 'g-')

        positions = []
        minCosts = []
        diffs = []
        chA = []
        chB = []
        chADiffs = []
        chBDiffs = []
        lastPos = None
        for d in self.data:
            pos, cost = calculatePosition(d[0], d[1], self.aVec, self.bVec)
            positions.append(pos)
            minCosts.append(cost)
            chA.append(self.aVec[pos])
            chB.append(self.bVec[pos])
            chADiffs.append(d[0] - self.aVec[pos])
            chBDiffs.append(d[1] - self.bVec[pos])
            if lastPos is None:
                lastPos = pos
            diff = pos - lastPos
            diffs.append(diff - int(round(diff / 2048)) * 2048)
            lastPos = pos

        fig = plt.figure(3)
        fig.suptitle('Encoder position')
        plt.plot(positions)

        fig = plt.figure(4)
        fig.suptitle('Min fitting cost')
        plt.plot(minCosts)

        fig = plt.figure(5)
        fig.suptitle('Encoder pfosition diff between samples')
        plt.plot(diffs)

        fig = plt.figure(6)
        fig.suptitle('Min fitting cost over encoder position')
        plt.plot(positions, minCosts, ',')

        fig = plt.figure(7)
        fig.suptitle('Position diff between samples over encoder position')
        plt.plot(positions, diffs, ',')

        a = self.data[:,0]
        b = self.data[:,1]
        c = []
        for d in zip(a, b):
            c.append(math.sqrt((d[0]**2 + d[1]**2) / 2))
        fig = plt.figure(8)
        fig.suptitle('Sensor values (red and green) and expected values from characteristic vectors')
        plt.plot(a, 'r')
        plt.plot(b, 'g')
        plt.plot(chA, 'm')
        plt.plot(chB, 'c')

        fig = plt.figure(9)
        fig.suptitle('Diff between real sensor values and expected values')
        plt.plot(chADiffs, 'r')
        plt.plot(chBDiffs, 'g')

        plt.show()

    def plotGeneratedVectors(self, box):
        fig = Figure(figsize=(5, 4), dpi=100)
        ax = fig.add_subplot()

        labelStr = ('Red is channel A and green is channel B. A good result\n'
                'is indicated by all points cohering to a smooth curve.')

        ax.plot(self.aVecShifted, 'r-')
        ax.plot(self.bVecShifted, 'g-')

        if self.oldAVec is not None and self.oldBVec is not None:
            ax.plot(self.oldAVec, 'm-')
            ax.plot(self.oldBVec, 'c-')

            labelStr += '\nMagenta and cyan curves are old channel A and channel B.'

        canvas = FigureCanvas(fig)
        canvas.set_size_request(600, 400)
        box.add(canvas)

        label = Gtk.Label(label=labelStr)
        label.set_use_markup(True)
        label.set_margin_start(30)
        label.set_margin_end(50)
        label.set_margin_top(8)
        label.set_margin_bottom(10)
        label.set_xalign(0.0)
        box.add(label)

        box.show_all()

    def writeVectorsToConfigFileString(self, configFileAsString, configClassName):
        configClassString = getConfigClassString(configFileAsString, configClassName)
        aVecPattern = OpticalEncoderDataVectorGenerator._aVecPattern
        bVecPattern = OpticalEncoderDataVectorGenerator._bVecPattern

        temp1 = aVecPattern.search(configClassString)
        temp2 = bVecPattern.search(configClassString)
        if temp1 is not None and temp2 is not None:
            configClassString = re.sub(aVecPattern, r'\g<beg>' + '2048' + r'\g<end>'
                    + intArrayToString(self.aVecShifted), configClassString)
            configClassString = re.sub(bVecPattern, r'\g<beg>' + '2048' + r'\g<end>'
                    + intArrayToString(self.bVecShifted), configClassString)

            configFileAsString = setConfigClassString(configFileAsString, configClassName, configClassString)
            return configFileAsString

        return ''

    def getGeneratedVectors(self):
        out = ''
        out += 'std::array<uint16_t, 2048 > aVec = ' + intArrayToString(self.aVecShifted) + '\n'
        out += 'std::array<uint16_t, 2048 > bVec = ' + intArrayToString(self.bVecShifted)

        return out

def createGuiBox(parent, nodeNr, getPortFun, configFilePath, configClassName):
    # pylint: disable=too-many-locals, too-many-statements
    calibrationBox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
    calibrationBox.set_margin_start(40)
    calibrationBox.set_margin_bottom(100)

    limitMovementButton = GuiFunctions.createToggleButton('Lock', getLowLev=True)
    limitMovementButton = (GuiFunctions.addTopLabelTo(
                '<b>Limit movement</b>\n Only move around locked position to avoid end limits',
                limitMovementButton[0]),
            limitMovementButton[1])
    calibrationBox.pack_start(limitMovementButton[0], False, False, 0)

    startPos = None

    def onLockPosition(widget):
        nonlocal startPos
        if widget.get_active():
            try:
                with createServoManager(nodeNr, getPortFun()) as servoManager:
                    startPos = servoManager.servoArray[0].getPosition(True)
            except Exception as e:
                GuiFunctions.exceptionMessage(parent, e)
                widget.set_active(False)

        else:
            startPos = None

    limitMovementButton[1].connect('toggled', onLockPosition)

    pwmValue = 0
    pwmScale = GuiFunctions.creatHScale(pwmValue, 0, 1023, 10, getLowLev=True)
    pwmScale = (GuiFunctions.addTopLabelTo(
                '<b>Motor pwm value</b>\n Choose a value that results in a moderate constant velocity', pwmScale[0]),
            pwmScale[1])
    calibrationBox.pack_start(pwmScale[0], False, False, 0)

    testButton = GuiFunctions.createButton('Test pwm value', getLowLev=True)
    calibrationBox.pack_start(testButton[0], False, False, 0)
    startButton = GuiFunctions.createButton('Start calibration', getLowLev=True)
    calibrationBox.pack_start(startButton[0], False, False, 0)

    recordingProgressBar = GuiFunctions.creatProgressBar(label='Recording', getLowLev=True)
    analyzingProgressBar = GuiFunctions.creatProgressBar(label='Analyzing', getLowLev=True)
    statusLabel = GuiFunctions.createLabel('')

    threadMutex = threading.Lock()
    def updatePwmValue(widget):
        nonlocal pwmValue

        with threadMutex:
            pwmValue = widget.get_value()

    pwmScale[1].connect('value-changed', updatePwmValue)

    def resetGuiAfterCalibration():
        testButton[1].set_label('Test pwm value')
        testButton[1].set_sensitive(True)
        startButton[1].set_label('Start calibration')
        startButton[1].set_sensitive(True)
        calibrationBox.remove(recordingProgressBar[0])
        calibrationBox.remove(analyzingProgressBar[0])
        calibrationBox.remove(statusLabel)
        pwmScale[1].set_sensitive(True)
        limitMovementButton[1].set_sensitive(True)

    def updateStatusLabel(statusStr):
        statusLabel.set_label(statusStr)

    runThread = False
    def testPwmRun(nodeNr, port):
        # pylint: disable=too-many-statements
        nonlocal runThread

        try:
            with createServoManager(nodeNr, port) as servoManager:
                t = 0.0
                doneRunning = False
                pwmDir = 1
                moveDir = 0

                def sendCommandHandlerFunction(dt, servoManager):
                    nonlocal pwmDir
                    nonlocal moveDir

                    servo = servoManager.servoArray[0]

                    pwm = 0
                    with threadMutex:
                        pwm = pwmValue

                    pos = servo.getPosition(True)

                    if startPos is not None:
                        if pos - startPos < -1 and moveDir != -1:
                            moveDir = -1
                            pwmDir *= -1
                        elif pos - startPos > 1 and moveDir != 1:
                            moveDir = 1
                            pwmDir *= -1

                    servo.setOpenLoopControlSignal(pwm * pwmDir, True)

                out = []

                def readResultHandlerFunction(dt, servoManager):
                    nonlocal t
                    nonlocal doneRunning

                    t += dt
                    servo = servoManager.servoArray[0]
                    opticalEncoderData = servo.getOpticalEncoderChannelData()
                    out.append([t,
                            opticalEncoderData.a,
                            opticalEncoderData.b,
                            opticalEncoderData.minCostIndex,
                            opticalEncoderData.minCost,
                            servo.getVelocity(),
                            servo.getPosition(True) / pi * 180.0])

                    GLib.idle_add(updateStatusLabel,
                            f'Sensor A: {opticalEncoderData.a}\n'
                            f'Sensor B: {opticalEncoderData.b}\n'
                            f'Encoder position: {opticalEncoderData.minCostIndex}\n'
                            f'Output Encoder position: {servo.getPosition(True) / pi * 180.0:0.2f}'
                    )

                    stop = False
                    with threadMutex:
                        if runThread is False:
                            stop = True

                    if stop or parent.isClosed:
                        servoManager.removeHandlerFunctions()
                        doneRunning = True

                servoManager.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction)

                while not doneRunning:
                    if not servoManager.isAlive():
                        break
                    time.sleep(0.1)

                servoManager.shutdown()

                data = np.array(out)

                def plotData(data):
                    dialog = Gtk.MessageDialog(
                            transient_for=parent,
                            flags=0,
                            message_type=Gtk.MessageType.INFO,
                            buttons=Gtk.ButtonsType.YES_NO,
                            text='Pwm test done!',
                    )
                    dialog.format_secondary_text(
                        "Do you want to plot the recorded data?"
                    )
                    response = dialog.run()
                    dialog.destroy()

                    if response == Gtk.ResponseType.YES:
                        fig = plt.figure(1)
                        fig.suptitle('Sensor A and B values')
                        plt.plot(data[:, 0], data[:, 1], 'r')
                        plt.plot(data[:, 0], data[:, 2], 'g')

                        fig = plt.figure(2)
                        fig.suptitle('Encoder position')
                        plt.plot(data[:, 0], data[:, 3])
                        fig = plt.figure(3)
                        fig.suptitle('Min fitting cost')
                        plt.plot(data[:, 0], data[:, 4])
                        fig = plt.figure(4)
                        fig.suptitle('Min fitting cost over encoder position')
                        plt.plot(data[:, 3], data[:, 4], 'r-')
                        plt.plot(data[:, 3], data[:, 4], 'g+')
                        fig = plt.figure(5)
                        fig.suptitle('Velocity over encoder position')
                        plt.plot(data[:, 3], data[:, 5], 'r-')
                        plt.plot(data[:, 3], data[:, 5], 'g+')

                        fig = plt.figure(6)
                        fig.suptitle('Sensor values over encoder position')
                        plt.plot(data[:, 3], data[:, 1], 'r')
                        plt.plot(data[:, 3], data[:, 2], 'g')

                        fig = plt.figure(7)
                        fig.suptitle('Output encoder position')
                        plt.plot(data[:, 0], data[:, 6])
                        plt.show()

                GLib.idle_add(plotData, data)

        except Exception as e:
            GuiFunctions.exceptionMessage(parent, e)
        finally:
            GLib.idle_add(resetGuiAfterCalibration)

    testThread = None
    def onTestPwm(widget):
        nonlocal testThread
        nonlocal runThread

        if widget.get_label() == 'Test pwm value':
            startButton[1].set_sensitive(False)
            limitMovementButton[1].set_sensitive(False)
            calibrationBox.pack_start(statusLabel, False, False, 0)

            calibrationBox.show_all()

            widget.set_label('Stop pwm test')
            with threadMutex:
                runThread = True
            testThread = threading.Thread(target=testPwmRun, args=(nodeNr, getPortFun(),))
            testThread.start()
        else:
            with threadMutex:
                runThread = False
            testThread.join()

    testButton[1].connect('clicked', onTestPwm)

    def updateRecordingProgressBar(fraction):
        recordingProgressBar[1].set_fraction(fraction)

    def updateAnalyzingProgressBar(fraction):
        analyzingProgressBar[1].set_fraction(fraction)

    def startCalibrationRun(nodeNr, port):
        # pylint: disable=too-many-locals, too-many-statements
        nonlocal runThread

        try:
            with createServoManager(nodeNr, port) as servoManager:

                def handleResults(opticalEncoderDataVectorGenerator):
                    dialog = Gtk.MessageDialog(
                            transient_for=parent,
                            flags=0,
                            message_type=Gtk.MessageType.INFO,
                            buttons=Gtk.ButtonsType.YES_NO,
                            text='Optical encoder calibration done!',
                    )
                    dialog.format_secondary_text(
                        "Do you want to plot extended data?"
                    )
                    response = dialog.run()
                    dialog.destroy()

                    if response == Gtk.ResponseType.YES:
                        opticalEncoderDataVectorGenerator.showAdditionalDiagnosticPlots()

                    dialog = Gtk.MessageDialog(
                            transient_for=parent,
                            flags=0,
                            message_type=Gtk.MessageType.INFO,
                            buttons=Gtk.ButtonsType.YES_NO,
                            text='Optical encoder calibration done!',
                    )
                    dialog.format_secondary_text(
                        "Should the configuration be updated with the new data?"
                    )
                    opticalEncoderDataVectorGenerator.plotGeneratedVectors(dialog.get_message_area())
                    dialog.get_widget_for_response(Gtk.ResponseType.YES).grab_focus()
                    response = dialog.run()
                    dialog.destroy()

                    if response == Gtk.ResponseType.YES:
                        with open(configFilePath, "r", encoding='utf-8') as configFile:
                            configFileAsString = configFile.read()
                            configFileAsString = opticalEncoderDataVectorGenerator.writeVectorsToConfigFileString(
                                    configFileAsString, configClassName)

                            if configFileAsString != '':
                                with open(configFilePath, "w", encoding='utf-8') as configFile:
                                    configFile.write(configFileAsString)
                                    GuiFunctions.transferToTargetMessage(parent)

                                return

                        dialog = Gtk.MessageDialog(
                                transient_for=parent,
                                flags=0,
                                message_type=Gtk.MessageType.ERROR,
                                buttons=Gtk.ButtonsType.OK,
                                text='Configuration format error!',
                        )
                        dialog.format_secondary_text(
                            "Please past in the new aVec and bVec manually"
                        )
                        box = dialog.get_message_area()
                        vecEntry = Gtk.Entry()
                        vecEntry.set_text(opticalEncoderDataVectorGenerator.getGeneratedVectors())
                        box.add(vecEntry)
                        box.show_all()
                        dialog.run()
                        dialog.destroy()

                def handleAnalyzeError(info):
                    dialog = Gtk.MessageDialog(
                            transient_for=parent,
                            flags=0,
                            message_type=Gtk.MessageType.ERROR,
                            buttons=Gtk.ButtonsType.OK,
                            text='Calibration failed during analyzing',
                    )
                    dialog.format_secondary_text(info)
                    dialog.run()
                    dialog.destroy()

                t = 0.0
                dirChangeWait = 0.0
                doneRunning = False
                pwmDir = 1
                moveDir = 0

                runTime = 110.0

                def sendCommandHandlerFunction(dt, servoManager):
                    nonlocal dirChangeWait
                    nonlocal pwmDir
                    nonlocal moveDir

                    servo = servoManager.servoArray[0]

                    pwm = 0
                    with threadMutex:
                        pwm = pwmValue

                    pos = servo.getPosition(True)

                    if startPos is not None:
                        if pos - startPos < -1:
                            dirChangeWait = 1.0
                            if moveDir != -1:
                                moveDir = -1
                                pwmDir *= -1
                        elif pos - startPos > 1:
                            dirChangeWait = 1.0
                            if moveDir != 1:
                                moveDir = 1
                                pwmDir *= -1

                    if t > (runTime - 10) * 0.5 and moveDir == 0:
                        moveDir = 1
                        pwmDir *= -1
                        dirChangeWait = 4.0

                    servo.setOpenLoopControlSignal(pwm * pwmDir * min(1.0, 0.25 * t), True)

                out = []

                def readResultHandlerFunction(dt, servoManager):
                    nonlocal t
                    nonlocal dirChangeWait
                    nonlocal doneRunning

                    servo = servoManager.servoArray[0]
                    opticalEncoderData = servo.getOpticalEncoderChannelData()
                    if t > 0.1 and dirChangeWait <= 0.0:
                        out.append([t,
                                opticalEncoderData.a,
                                opticalEncoderData.b,
                                opticalEncoderData.minCostIndex,
                                opticalEncoderData.minCost])

                    GLib.idle_add(updateRecordingProgressBar, t / runTime)

                    stop = t >= runTime
                    with threadMutex:
                        if runThread is False:
                            stop = True

                    if stop or parent.isClosed:
                        servoManager.removeHandlerFunctions()
                        doneRunning = True

                    if dirChangeWait > 0.0:
                        dirChangeWait -= dt
                    else:
                        t += dt

                servoManager.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction)

                while not doneRunning:
                    if not servoManager.isAlive():
                        runThread = False
                        break
                    time.sleep(0.1)

                data = np.array(out)

                servoManager.shutdown()

                def shouldAbort():

                    with threadMutex:
                        if runThread is False:
                            return True
                    return False

                lastFraction = 0.0
                def updateProgress(fraction):
                    nonlocal lastFraction

                    if abs(fraction - lastFraction) > 0.01:
                        lastFraction = fraction
                        GLib.idle_add(updateAnalyzingProgressBar, fraction)

                if not shouldAbort():
                    try:
                        configFileAsString = ''
                        with open(configFilePath, "r", encoding='utf-8') as configFile:
                            configFileAsString = configFile.read()

                        opticalEncoderDataVectorGenerator = OpticalEncoderDataVectorGenerator(
                                data[:, 1:3], configFileAsString, configClassName, constVelIndex=4000,
                                segment=2 * 2048, noiseDepresMemLenght=8,
                                shouldAbort=shouldAbort,
                                updateProgress=updateProgress)

                        GLib.idle_add(handleResults, opticalEncoderDataVectorGenerator)
                    except Exception as e:
                        GLib.idle_add(handleAnalyzeError, format(e))

        except Exception as e:
            GuiFunctions.exceptionMessage(parent, e)
        finally:
            GLib.idle_add(resetGuiAfterCalibration)

    def onStartCalibration(widget):
        nonlocal testThread
        nonlocal runThread

        if widget.get_label() == 'Start calibration':
            widget.set_label('Abort calibration')

            recordingProgressBar[1].set_fraction(0.0)
            analyzingProgressBar[1].set_fraction(0.0)
            testButton[1].set_sensitive(False)
            calibrationBox.pack_start(recordingProgressBar[0], False, False, 0)
            calibrationBox.pack_start(analyzingProgressBar[0], False, False, 0)
            pwmScale[1].set_sensitive(False)
            limitMovementButton[1].set_sensitive(False)

            calibrationBox.show_all()

            with threadMutex:
                runThread = True
            testThread = threading.Thread(target=startCalibrationRun, args=(nodeNr, getPortFun(),))
            testThread.start()
        else:
            with threadMutex:
                runThread = False
            testThread.join()

    startButton[1].connect('clicked', onStartCalibration)

    calibrationBox.show_all()
    return calibrationBox
