'''
test code for CalibrationAnalyzers Helper
'''

from unittest import TestCase
from ServoProjectModules.CalibrationAnalyzers.SystemIdentification import * # pylint: disable=wildcard-import, unused-wildcard-import

class Tester(TestCase):
    def testSmoothMoveHandler(self):
        data = np.loadtxt('tests/dataFiles/sysTestDataSimT02.txt')
        systemIdentifier = SystemIdentificationObject(data)
        simulationError = systemIdentifier.simError[:]
        maxVel = max(abs(v) for v in systemIdentifier.simVel)
        simulationError.sort()
        l = len(simulationError)
        absRelativeError = [abs(e) / maxVel for e in simulationError[l // 100: -l // 100]]
        self.assertLess(max(absRelativeError), 0.01)
