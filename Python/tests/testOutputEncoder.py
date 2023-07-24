'''
test code for CalibrationAnalyzers Helper
'''

from unittest import TestCase
from ServoProjectModules.CalibrationAnalyzers.OutputEncoder import * # pylint: disable=wildcard-import, unused-wildcard-import

class Tester(TestCase):
    def test(self):
        data = np.loadtxt('tests/dataFiles/outputEncoderData.txt')
        OutputEncoderCalibrationGenerator(data, False, 4096.0 * 220.0 / 360)
