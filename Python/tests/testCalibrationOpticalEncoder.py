'''
test code for CalibrationAnalyzers Helper
'''

from unittest import TestCase
from ServoProjectModules.CalibrationAnalyzers.OpticalEncoder import * # pylint: disable=wildcard-import, unused-wildcard-import

class Tester(TestCase):
    def testSmoothMoveHandler(self):
        data = np.loadtxt('tests/dataFiles/optEncTestData.txt')

        opticalEncoderDataVectorGenerator = OpticalEncoderDataVectorGenerator(
                data, constVelIndex=4000)

        _, _, _, minCost, _ = (
                opticalEncoderDataVectorGenerator.getAdditionalDiagnostics())

        minCost.sort()
        l = len(minCost)
        self.assertLess(minCost[-l // 50], 100)
