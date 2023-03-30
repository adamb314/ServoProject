'''
test code for CalibrationAnalyzers Helper
'''

from unittest import TestCase
from ServoProjectModules.CalibrationAnalyzers.Helper import * # pylint: disable=wildcard-import, unused-wildcard-import

class Tester(TestCase):
    def testSmoothMoveHandler(self):
        moveHandler = SmoothMoveHandler(0.0, 0.4)
        moveHandler.set(0.0, 0.2)

        dt = 0.01
        refV = 1.0
        r = 1.0
        moveHandler.set(r, refV)

        p, v = moveHandler.getNextRef(0.0)

        pVec = [p]
        vVec = [v]
        rVec = [r]

        self.assertEqual(pVec[-1], 0.0)

        for i in range(0, 100):
            p, v = moveHandler.getNextRef(dt)
            pVec.append(p)
            vVec.append(v)
            rVec.append(r)

        r = 1.5

        for i in range(0, 100):
            moveHandler.set(r, refV)
            p, v = moveHandler.getNextRef(dt)
            pVec.append(p)
            vVec.append(v)
            rVec.append(r)

        for i in range(0, 500):
            r = 0.5 + min(0.3, i * 0.001)
            moveHandler.set(r, refV)
            p, v = moveHandler.getNextRef(dt)
            pVec.append(p)
            vVec.append(v)
            rVec.append(r)

        velErrors = []
        for i, d in enumerate(zip(pVec[1:], pVec[0:-1], vVec[1:], vVec[0:-1])):
            actVel = (d[0] - d[1]) / dt
            meanVel = (d[2] + d[3]) / 2
            velErrors.append(abs(actVel - meanVel))

        self.assertLess(max(velErrors[0:300]), 0.0025)
        self.assertLess(max(velErrors[300:375]), 0.025)
        self.assertLess(max(velErrors[375:]), 0.0025)
        self.assertEqual(max(pVec), 1.5)
        self.assertEqual(round(max(vVec) * 1000), round(refV * 1000))
        self.assertEqual(round(min(vVec) * 1000), round(-refV * 1000))
        self.assertEqual(pVec[-1], rVec[-1])

        #plt.figure(1)
        #plt.plot(pVec, 'g+')
        #plt.plot(vVec, 'y+')
        #plt.plot(actVel, 'c+')
        #plt.plot(rVec, 'r+')
        #plt.figure(2)
        #plt.plot(velErrors)
        #plt.show()

    def testPiecewiseLinearFunction(self):
        with self.assertRaises(Exception) as context:
            f = PiecewiseLinearFunction([1], [2, 3])
        self.assertEqual(str(Exception('x and y list not same length')), str(context.exception))

        with self.assertRaises(Exception) as context:
            f = PiecewiseLinearFunction([1], [2])
        self.assertEqual(str(Exception('x list is too short')), str(context.exception))

        with self.assertRaises(Exception) as context:
            f = PiecewiseLinearFunction([1, 2, 1], [2, 3, 3])
        self.assertEqual(str(Exception('x list is not sorted')), str(context.exception))

        with self.assertRaises(Exception) as context:
            f = PiecewiseLinearFunction([1, 2, 3], [2, 3, 2])
            f.getX(2)
        self.assertEqual(str(Exception('not a monotone function')), str(context.exception))

        f = PiecewiseLinearFunction([1, 2], [2, 3])
        self.assertEqual(f.getY(1.75), 2.75)

        f = PiecewiseLinearFunction([1, 2], [3, 2])
        self.assertEqual(f.getY(0.75), 3.25)
        self.assertEqual(f.getY(1.75), 2.25)
        self.assertEqual(f.getY(2.25), 1.75)
        self.assertEqual(f.getY(f.getX(2.25)), 2.25)

        f = PiecewiseLinearFunction([-1, 0, 1, 2, 3], [3, 3, 3, 2, 1.5])
        self.assertEqual(f.getY(0.75), 3.0)
        self.assertEqual(f.getY(1.75), 2.25)
        self.assertEqual(f.getY(2.5), 1.75)
