from ServoProjectModules.CalibrationAnalyzers.Helper import *

@numba.jit(nopython=True)
def calcCovWithEndOfVectors(aVec, bVec, ca, cb, noiseDepresMemLenght,  dir):
    cov = 0

    aDiff = (aVec[-1] - aVec[-noiseDepresMemLenght]) / noiseDepresMemLenght
    bDiff = (bVec[-1] - bVec[-noiseDepresMemLenght]) / noiseDepresMemLenght

    if dir == False:
        aDiff = 0
        bDiff = 0

    for i, (a, b) in enumerate(zip(aVec[-noiseDepresMemLenght:], bVec[-noiseDepresMemLenght:])):
        a += aDiff * (noiseDepresMemLenght - i - 1)
        b += bDiff * (noiseDepresMemLenght - i - 1)

        cov += (ca - a)**2 + (cb - b)**2
    if dir == True:
        cov += noiseDepresMemLenght * (
            (ca - aVec[-noiseDepresMemLenght+1] -
                (aVec[-1] - aVec[-noiseDepresMemLenght]))**2 +
            (cb - bVec[-noiseDepresMemLenght+1] -
                (bVec[-1] - bVec[-noiseDepresMemLenght]))**2)
        
    return cov

@numba.jit(nopython=True)
def findBestFitt(data, modData, aVec, bVec, noiseDepresMemLenght):
    minCov = 1000000
    minIndex = 0
    done = False
    wrapIndex = 0

    for i, d in enumerate(modData):
        cov = calcCovWithEndOfVectors(aVec, bVec, d[0], d[1], noiseDepresMemLenght, False)

        if minCov > cov:
            minIndex = i
            minCov = cov

    if len(modData) < len(data) * 0.2:
        for i in range(noiseDepresMemLenght, int(len(aVec) / 2)):
            cov = calcCovWithEndOfVectors(aVec, bVec, aVec[i], bVec[i], noiseDepresMemLenght, False)

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

    for i, d in enumerate(zip(aVec[1:], bVec[1:])):
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
def calcFitCov(i, a, b, aVec, bVec, noiseDepresMemLenght):
    cov = 0
    iMin = i - noiseDepresMemLenght
    iMax = i + 2 + noiseDepresMemLenght
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
        a = aVec[i + 1]
        b = bVec[i + 1]

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
    def __init__(self, data, classString = '', segment = 3000,
            constVelIndex = 10000, noiseDepresMemLenght = 5,
            shouldAbort=None, updateProgress=None):

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

            if startOfFirstRotation == None:
                if c > meanCost:
                    startOfFirstRotation = i
            elif endOfFirstRotation == None:
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

        if startOfFirstRotation == None or endOfFirstRotation == None:
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
            if self.shouldAbort != None:
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
        for i, d in enumerate(zip(self.aVecList[1:], 
                self.bVecList[1:])):
            tempAVec = numba.typed.List(d[0])
            tempBVec = numba.typed.List(d[1])

            initialTempVecLenght = len(tempAVec)
            while len(tempAVec) > 0:
                minIndex = findBestFitt2Opt(tempAVec[0], tempBVec[0], mergedAVectors, mergedBVectors, 2 * i + self.noiseDepresMemLenght)
                inserts[minIndex].append((tempAVec[0], tempBVec[0]))
                del tempAVec[0]
                del tempBVec[0]

                if self.updateProgress != None:
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

        if classString != '':
            aVecPattern = re.compile(r'(?P<beg>.*createMainEncoderHandler\(\)\s*\{(?:.*\n)*?\s*std\s*::\s*array\s*<\s*uint16_t\s*,\s*)\d+(?P<end>\s*>\s*aVec\s*=\s*)\{([\d\s,]*)\};')
            bVecPattern = re.compile(r'(?P<beg>.*createMainEncoderHandler\(\)\s*\{(?:.*\n)*?\s*std\s*::\s*array\s*<\s*uint16_t\s*,\s*)\d+(?P<end>\s*>\s*bVec\s*=\s*)\{([\d\s,]*)\};')
            
            temp1 = aVecPattern.search(classString)
            temp2 = bVecPattern.search(classString)

            if temp1 != None and temp2 != None:
                oldAVecStr = temp1.group(3)
                oldBVecStr = temp2.group(3)
                oldAVecStr = '[' + oldAVecStr + ']'
                oldBVecStr = '[' + oldBVecStr + ']'

                self.oldAVec = eval(oldAVecStr)
                self.oldBVec = eval(oldBVecStr)

                if len(self.oldAVec) <= 1:
                    self.oldAVec = None
                if len(self.oldBVec) <= 1:
                    self.oldBVec = None

        if self.oldAVec != None and self.oldBVec != None:
            bestFittShift = getShift(self.aVec, self.bVec, self.oldAVec, self.oldBVec)
            self.aVecShifted = shiftVec(self.aVec, bestFittShift)
            self.bVecShifted = shiftVec(self.bVec, bestFittShift)
        else:
            self.aVecShifted = self.aVec
            self.bVecShifted = self.bVec

    def genVec(self, data, partOfTotal = 1.0, donePrev = 0.0):
        aVec = numba.typed.List()
        bVec = numba.typed.List()
        for i in range(0, self.noiseDepresMemLenght):
            aVec.append(self.a0)
            bVec.append(self.b0)

        modData = data[:]
        wrapIndex = 0

        while len(modData) > 0:
            if self.shouldAbort != None:
                if self.shouldAbort():
                    return

            if self.updateProgress != None:
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
            if self.shouldAbort != None:
                if self.shouldAbort():
                    return

            if self.updateProgress != None:
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
        sensorValueOffset = 0
        predictNextPos = 0
        iAtLastOffsetUpdate = 0
        lastMinCostIndex = 0

        def calcCost(i, a, b, aVec, bVec):
            vecSize = len(aVec)

            if (i >= vecSize):
                i -= vecSize
            elif (i < 0):
                i += vecSize

            tempA = aVec[i] - a
            tempA = tempA * tempA

            tempB = bVec[i] - b
            tempB = tempB * tempB

            return (tempA + tempB, i)

        def calculatePosition(ca, cb, aVec, bVec):
            nonlocal sensorValueOffset
            nonlocal predictNextPos
            nonlocal iAtLastOffsetUpdate
            nonlocal lastMinCostIndex

            sensor1Value = ca
            sensor2Value = cb
            vecSize = len(aVec)

            #int16_t offset = sensorValueOffset
            diagnosticData_a = sensor1Value
            diagnosticData_b = sensor2Value
            #sensor1Value -= offset
            #sensor2Value -= offset

            stepSize = int(vecSize / 2.0 + 1)

            i = predictNextPos
            cost, i = calcCost(i, sensor1Value, sensor2Value, aVec, bVec)

            bestI = i
            bestCost = cost

            i += stepSize
            while (i < vecSize):
                cost, i = calcCost(i, sensor1Value, sensor2Value, aVec, bVec)

                if (cost < bestCost):
                    bestI = i
                    bestCost = cost

                i += stepSize

            checkDir = 1

            while not stepSize == 1:
                i = bestI

                if (stepSize <= 4):
                    stepSize -= 1
                else:
                    stepSize = int(stepSize / 2)
                    if (stepSize == 0):
                        stepSize = 1

                i += stepSize * checkDir

                cost, i = calcCost(i, sensor1Value, sensor2Value, aVec, bVec)

                if (cost < bestCost):
                    bestI = i
                    bestCost = cost

                    checkDir *= -1

                    continue

                i -= 2 * stepSize * checkDir

                cost, i = calcCost(i, sensor1Value, sensor2Value, aVec, bVec)

                if (cost < bestCost):
                    bestI = i
                    bestCost = cost

            bestI = int(bestI)

            if (abs(bestI - iAtLastOffsetUpdate) > (vecSize / 10)):
                iAtLastOffsetUpdate = bestI

                a = aVec[bestI]
                b = bVec[bestI]
                currentOffset = (diagnosticData_a - a + diagnosticData_b - b) / 2

                sensorValueOffset = 0.95 * sensorValueOffset + 0.05 * currentOffset

            lastMinCostIndexChange = bestI - lastMinCostIndex
            lastMinCostIndex = bestI
            predictNextPos = bestI + lastMinCostIndexChange
            while (predictNextPos >= vecSize):
                predictNextPos -= vecSize
            while (predictNextPos < 0):
                predictNextPos += vecSize

            return (bestI, bestCost)

        plt.figure(1)
        plt.plot(self.mergedAVectors, 'r-')
        plt.plot(self.mergedBVectors, 'g-')

        plt.figure(2)
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
            if lastPos == None:
                lastPos = pos
            diff = pos - lastPos
            diffs.append(diff - int(round(diff / 2048)) * 2048)
            lastPos = pos

        plt.figure(3)
        plt.plot(positions)

        plt.figure(4)
        plt.plot(minCosts)

        plt.figure(5)
        plt.plot(diffs)

        plt.figure(6)
        plt.plot(positions, minCosts, ',')

        plt.figure(7)
        plt.plot(positions, diffs, ',')

        a = self.data[:,0]
        b = self.data[:,1]
        c = []
        for d in zip(a, b):
            c.append(math.sqrt((d[0]**2 + d[1]**2) / 2))
        plt.figure(8)
        plt.plot(a, 'r')
        plt.plot(b, 'g')
        plt.plot(chA, 'm')
        plt.plot(chB, 'c')

        plt.figure(9)
        plt.plot(chADiffs, 'r')
        plt.plot(chBDiffs, 'g')

        plt.show()

    def plotGeneratedVectors(self, box):
        fig = Figure(figsize=(5, 4), dpi=100)
        ax = fig.add_subplot()

        labelStr = 'Red is channel A and green is channel B. A good result\nis indicated by all points cohering to a smooth curve.'
        
        self.showAdditionalDiagnosticPlots()

        ax.plot(self.aVecShifted, 'r-')
        ax.plot(self.bVecShifted, 'g-')

        if self.oldAVec != None and self.oldBVec != None:
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

    def writeVectorsToConfigClassString(self, configClassString):
        aVecPattern = re.compile(r'(?P<beg>.*createMainEncoderHandler\(\)\s*\{(?:.*\n)*?\s*std\s*::\s*array\s*<\s*uint16_t\s*,\s*)\d+(?P<end>\s*>\s*aVec\s*=\s*)\{[\d\s,]*\};')
        bVecPattern = re.compile(r'(?P<beg>.*createMainEncoderHandler\(\)\s*\{(?:.*\n)*?\s*std\s*::\s*array\s*<\s*uint16_t\s*,\s*)\d+(?P<end>\s*>\s*bVec\s*=\s*)\{[\d\s,]*\};')
        
        temp1 = aVecPattern.search(configClassString)
        temp2 = bVecPattern.search(configClassString)
        if temp1 != None and temp2 != None:
            configClassString = re.sub(aVecPattern, r'\g<beg>' + '2048' + r'\g<end>' + intArrayToString(self.aVecShifted), configClassString)
            configClassString = re.sub(bVecPattern, r'\g<beg>' + '2048' + r'\g<end>' + intArrayToString(self.bVecShifted), configClassString)

            return configClassString

        return ''

    def getGeneratedVectors(self):
        out = ''
        out += 'std::array<uint16_t, 2048 > aVec = ' + intArrayToString(self.aVecShifted) + '\n'
        out += 'std::array<uint16_t, 2048 > bVec = ' + intArrayToString(self.bVecShifted)

        return out

def createGuiBox(parent, nodeNr, port, configFilePath, configClassName, manualMovement):
    calibrationBox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
    calibrationBox.set_margin_start(40)
    pwmValue = 0
    if not manualMovement:
        pwmValue = 1
    pwmScale = GuiFunctions.creatHScale(pwmValue, 0, 1023, 10, getLowLev=True)
    pwmScale = GuiFunctions.addTopLabelTo('<b>Motor pwm value</b>\n Choose a value that results in a moderate constant velocity', pwmScale[0]), pwmScale[1]
    if not manualMovement:
        calibrationBox.pack_start(pwmScale[0], False, False, 0)

    testButton = GuiFunctions.createButton('Test pwm value', getLowLev=True)
    startButton = GuiFunctions.createButton('Start calibration', getLowLev=True)

    threadMutex = threading.Lock()
    def updatePwmValue(widget):
        nonlocal pwmValue
        nonlocal threadMutex

        with threadMutex:
            pwmValue = widget.get_value()

    pwmScale[1].connect('value-changed', updatePwmValue)

    def resetGuiAfterCalibration():
        nonlocal startButton
        nonlocal testButton
        nonlocal calibrationBox
        nonlocal recordingProgressBar
        nonlocal analyzingProgressBar
        nonlocal pwmScale

        testButton[1].set_label('Test pwm value')
        testButton[1].set_sensitive(True)
        startButton[1].set_label('Start calibration')
        startButton[1].set_sensitive(True)
        calibrationBox.remove(recordingProgressBar[0])
        calibrationBox.remove(analyzingProgressBar[0])
        pwmScale[1].set_sensitive(True)

    runThread = False
    def testPwmRun(nodeNr, port):
        nonlocal runThread
        nonlocal pwmValue
        nonlocal threadMutex
        
        try:
            robot = createRobot(nodeNr, port)

            t = 0.0
            doneRunning = False

            def sendCommandHandlerFunction(dt, robot):
                nonlocal t
                nonlocal pwmValue
                nonlocal threadMutex

                servo = robot.dcServoArray[0]

                pwm = 0
                with threadMutex:
                    pwm = pwmValue

                servo.setOpenLoopControlSignal(pwm, True)

            out = []

            def readResultHandlerFunction(dt, robot):
                nonlocal t
                nonlocal runThread
                nonlocal doneRunning

                t += dt
                servo = robot.dcServoArray[0]
                opticalEncoderData = servo.getOpticalEncoderChannelData()
                out.append([t,
                        opticalEncoderData.a,
                        opticalEncoderData.b,
                        opticalEncoderData.minCostIndex,
                        opticalEncoderData.minCost,
                        servo.getVelocity()])

                stop = False
                with threadMutex:
                    if runThread == False:
                        stop = True

                if stop or parent.isClosed:
                    robot.removeHandlerFunctions()
                    doneRunning = True

            robot.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction);

            while not doneRunning:
                time.sleep(0.1)

            robot.shutdown()

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
                    plt.figure(1)
                    plt.plot(data[:, 0], data[:, 1], 'r')
                    plt.plot(data[:, 0], data[:, 2], 'g')
                    
                    plt.figure(2)
                    plt.plot(data[:, 0], data[:, 3])
                    plt.figure(3)
                    plt.plot(data[:, 0], data[:, 4])
                    plt.figure(4)
                    plt.plot(data[:, 3], data[:, 4], 'r-')
                    plt.plot(data[:, 3], data[:, 4], 'g+')
                    plt.figure(5)
                    plt.plot(data[:, 3], data[:, 5], 'r-')
                    plt.plot(data[:, 3], data[:, 5], 'g+')

                    plt.figure(6)
                    plt.plot(data[:, 3], data[:, 1], 'r')
                    plt.plot(data[:, 3], data[:, 2], 'g')
                    plt.show()

            GLib.idle_add(plotData, data)

        except Exception as e:
            print(format(e))

        GLib.idle_add(resetGuiAfterCalibration)

    testThread = None

    def onTestPwm(widget):
        nonlocal calibrationBox
        nonlocal startButton

        nonlocal testThread
        nonlocal runThread
        nonlocal pwmValue
        nonlocal threadMutex

        if widget.get_label() == 'Test pwm value':
            startButton[1].set_sensitive(False)
            widget.set_label('Stop pwm test')
            with threadMutex:
                runThread = True
            testThread = threading.Thread(target=testPwmRun, args=(nodeNr, port,))
            testThread.start()
        else:
            with threadMutex:
                runThread = False
            testThread.join()


    testButton[1].connect('clicked', onTestPwm)
    if not manualMovement:
        calibrationBox.pack_start(testButton[0], False, False, 0)

    recordingProgressBar = GuiFunctions.creatProgressBar(label='Recording', getLowLev=True)
    analyzingProgressBar = GuiFunctions.creatProgressBar(label='Analyzing', getLowLev=True)

    def updateRecordingProgressBar(fraction):
        nonlocal recordingProgressBar

        recordingProgressBar[1].set_fraction(fraction)

    def updateAnalyzingProgressBar(fraction):
        nonlocal analyzingProgressBar

        analyzingProgressBar[1].set_fraction(fraction)
    
    def startCalibrationRun(nodeNr, port):
        nonlocal configClassName

        nonlocal runThread
        nonlocal pwmValue
        nonlocal threadMutex

        configClassString = ''

        with open(configFilePath, "r") as configFile:
            configFileAsString = configFile.read()

            classPattern =re.compile(r'(\s*class\s+' + configClassName + r'\s+(.*\n)*?(.*\n)*?\})')
            temp = classPattern.search(configFileAsString)
            if temp != None:
                configClassString = temp.group(0)

        try:
            robot = createRobot(nodeNr, port)

            def handleResults(opticalEncoderDataVectorGenerator):
                nonlocal configClassString
                nonlocal configFileAsString

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
                response = dialog.run()
                dialog.destroy()

                if response == Gtk.ResponseType.YES:
                    configClassString = opticalEncoderDataVectorGenerator.writeVectorsToConfigClassString(configClassString)
                    if configClassString != '':
                        configFileAsString = re.sub(classPattern, configClassString, configFileAsString)
                        with open(configFilePath, "w") as configFile:
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
                    response = dialog.run()
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
                response = dialog.run()
                dialog.destroy()

            t = 0.0
            doneRunning = False

            runTime = 110.0
            if manualMovement:
                runTime = 40.0

            def sendCommandHandlerFunction(dt, robot):
                nonlocal t
                nonlocal pwmValue
                nonlocal threadMutex
                nonlocal runTime

                servo = robot.dcServoArray[0]

                if manualMovement:
                    return

                pwm = 0
                with threadMutex:
                    pwm = pwmValue
                if t < (runTime - 10.0) * 0.5:
                    servo.setOpenLoopControlSignal(min(pwm, 0.25 * t * pwm), True)
                else:
                    servo.setOpenLoopControlSignal(-pwm, True)


            out = []

            def readResultHandlerFunction(dt, robot):
                nonlocal t
                nonlocal runThread
                nonlocal doneRunning
                nonlocal runTime

                servo = robot.dcServoArray[0]
                opticalEncoderData = servo.getOpticalEncoderChannelData()
                if t > 0.1 and (t < (runTime - 10.0) * 0.5 or t > (runTime - 10.0) * 0.5 + 10.0 or manualMovement):
                    out.append([t,
                            opticalEncoderData.a,
                            opticalEncoderData.b,
                            opticalEncoderData.minCostIndex,
                            opticalEncoderData.minCost])

                GLib.idle_add(updateRecordingProgressBar, t / runTime)

                stop = t >= runTime
                with threadMutex:
                    if runThread == False:
                        stop = True

                if stop or parent.isClosed:
                    robot.removeHandlerFunctions()
                    doneRunning = True

                t += dt

            robot.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction)

            while not doneRunning:
                time.sleep(0.1)

            data = np.array(out)

            robot.shutdown()

            def shouldAbort():
                nonlocal runThread
                nonlocal threadMutex

                with threadMutex:
                    if runThread == False:
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
                    opticalEncoderDataVectorGenerator = OpticalEncoderDataVectorGenerator(
                            data[:, 1:3], configClassString, constVelIndex=4000, segment = 2 * 2048, noiseDepresMemLenght= 8,
                            shouldAbort=shouldAbort,
                            updateProgress=updateProgress)

                    GLib.idle_add(handleResults, opticalEncoderDataVectorGenerator)
                except Exception as e:
                    GLib.idle_add(handleAnalyzeError, format(e))

        except Exception as e:
            print(format(e))

        GLib.idle_add(resetGuiAfterCalibration)
    
    def onStartCalibration(widget):
        nonlocal testThread
        nonlocal threadMutex
        nonlocal runThread

        nonlocal calibrationBox
        nonlocal pwmScale
        nonlocal recordingProgressBar
        nonlocal analyzingProgressBar
        nonlocal testButton

        if widget.get_label() == 'Start calibration':
            widget.set_label('Abort calibration')

            if manualMovement:
                GuiFunctions.startManuallyCalibrationMessage(parent, '200 seconds')

            recordingProgressBar[1].set_fraction(0.0)
            analyzingProgressBar[1].set_fraction(0.0)
            testButton[1].set_sensitive(False)
            calibrationBox.pack_start(recordingProgressBar[0], False, False, 0)
            calibrationBox.pack_start(analyzingProgressBar[0], False, False, 0)
            pwmScale[1].set_sensitive(False)

            calibrationBox.show_all()

            with threadMutex:
                runThread = True
            testThread = threading.Thread(target=startCalibrationRun, args=(nodeNr, port,))
            testThread.start()                                    
        else:
            with threadMutex:
                runThread = False
            testThread.join()


    startButton[1].connect('clicked', onStartCalibration)
    calibrationBox.pack_start(startButton[0], False, False, 0)

    calibrationBox.show_all()
    return calibrationBox
