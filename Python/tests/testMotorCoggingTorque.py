'''
test code for CalibrationAnalyzers Helper
'''

from unittest import TestCase
from ServoProjectModules.CalibrationAnalyzers.MotorCoggingTorque import * # pylint: disable=wildcard-import, unused-wildcard-import

class Tester(TestCase):
    def test(self):
        data = np.loadtxt('tests/dataFiles/motorCoggingTorque.txt')
        CoggingTorqueCalibrationGenerator(data)
