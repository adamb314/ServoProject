'''
test code for ServoProjectModule Communication
'''

from unittest import TestCase
from ServoProjectModules.Communication import * # pylint: disable=wildcard-import, unused-wildcard-import

class Tester(TestCase):
    def testIntCastingFunctions(self):
        self.assertEqual(removeIntWraparound(10, 2**16 - 4, bitLenght=16), 2**16 + 10)
        self.assertEqual(removeIntWraparound(2**16 -5, 11, bitLenght=16), -5)
        self.assertEqual(removeIntWraparound(10, 2**15 + 10, bitLenght=16), 2**16 + 10)
        self.assertEqual(removeIntWraparound(2**15 + 9, 10, bitLenght=16), 2**15 + 9)
        self.assertEqual(removeIntWraparound(2**16 + 2**15 + 9, 2**16 + 10, bitLenght=16), 2**16 + 2**15 + 9)
        self.assertEqual(removeIntWraparound(2**15 + 12, 10, bitLenght=16), -2**15 + 12)

        self.assertEqual(toUnsignedChar(10), 10)
        self.assertEqual(toUnsignedChar(-10), 2**8 - 10)
        self.assertEqual(toUnsignedChar(-3 * 2**8 + 10), 10)

        self.assertEqual(toUnsignedInt16(1000), 1000)
        self.assertEqual(toUnsignedInt16(-1000), 2**16 - 1000)
        self.assertEqual(toUnsignedInt16(-3 * 2**16 + 1000), 1000)

        self.assertEqual(unsignedToSignedInt(toUnsignedInt16(1000)), 1000)
        self.assertEqual(unsignedToSignedInt(toUnsignedInt16(-1000)), -1000)

        self.assertEqual(unsignedToSignedChar(toUnsignedChar(100)), 100)
        self.assertEqual(unsignedToSignedChar(toUnsignedChar(-100)), -100)

    def testComDelayInt(self):
        comDelayInt = ComDelayInt(delay=5, initValue=0)
        comDelayInt.setRight(1)
        self.assertEqual(0, comDelayInt.getLeft())
        comDelayInt.execute()
        comDelayInt.setRight(2)
        self.assertEqual(0, comDelayInt.getLeft())
        comDelayInt.execute()
        comDelayInt.setRight(3)
        self.assertEqual(0, comDelayInt.getLeft())
        comDelayInt.execute()
        comDelayInt.setRight(4)
        self.assertEqual(0, comDelayInt.getLeft())
        comDelayInt.execute()
        comDelayInt.setRight(5)
        self.assertEqual(1, comDelayInt.getLeft())

        comDelayInt.setLeft(1)
        self.assertEqual(5, comDelayInt.getRight())
        comDelayInt.execute()
        comDelayInt.setLeft(2)
        self.assertEqual(4, comDelayInt.getRight())
        comDelayInt.execute()
        comDelayInt.setLeft(3)
        self.assertEqual(3, comDelayInt.getRight())
        comDelayInt.execute()
        comDelayInt.setLeft(4)
        self.assertEqual(2, comDelayInt.getRight())
        comDelayInt.execute()
        comDelayInt.setLeft(5)
        self.assertEqual(1, comDelayInt.getRight())

    def testRawCommunication(self):
        com = SimulateCommunication()
        com.setNodeNr(1)
        com.execute()

        com.requestReadChar(2)
        com.requestReadChar(4)
        com.requestReadChar(5)
        com.requestReadInt(2)
        com.requestReadInt(4)
        com.requestReadInt(5)
        com.execute()
        self.assertEqual(0, com.getLastReadChar(2))
        self.assertEqual(0, com.getLastReadChar(4))
        self.assertEqual(0, com.getLastReadChar(5))
        self.assertEqual(0, com.getLastReadInt(2))
        self.assertEqual(0, com.getLastReadInt(4))
        self.assertEqual(0, com.getLastReadInt(5))

        com.writeChar(2, 100)
        com.writeInt(2, 1000)
        com.requestReadChar(2)
        com.requestReadInt(2)
        com.execute()
        self.assertEqual(0, com.getLastReadChar(2))
        self.assertEqual(0, com.getLastReadInt(2))

        com.requestReadChar(2)
        com.requestReadInt(2)
        com.execute()
        self.assertEqual(100, com.getLastReadChar(2))
        self.assertEqual(1000, com.getLastReadInt(2))

    def testDCServo(self):
        com = SimulateCommunication()
        servo = DCServoCommunicator(1, com)

        while not servo.isInitComplete():
            servo.run()
            self.assertEqual(1257.5, servo.getPosition())

        servo.setReference(1200, 1000, 500)
        servo.run()
        self.assertEqual(1257.5, servo.getPosition())
        self.assertEqual(0, servo.getVelocity())
        self.assertEqual(0, servo.getFeedforwardU())

        servo.setReference(1200, 1000, 500)
        servo.run()
        self.assertEqual(1257.5, servo.getPosition())
        self.assertEqual(0, servo.getVelocity())
        self.assertEqual(0, servo.getFeedforwardU())

        servo.setReference(1200, 1000, 500)
        servo.run()
        self.assertEqual(1200, servo.getPosition())
        self.assertEqual(1000, servo.getVelocity())
        self.assertEqual(500, servo.getFeedforwardU())

        servo.setOffsetAndScaling(180 / 2048, -90)
        self.assertEqual(1200 * 180 / 2048 - 90, servo.getPosition())
        self.assertEqual(1000 * 180 / 2048, servo.getVelocity())
        self.assertEqual(500, servo.getFeedforwardU())
