#!/bin/python3

'''
Python demo for controlling ServoProject servos
'''

import time
import math

import numpy as np
from ServoProjectModules.Communication import pi

import ServoProjectModules.Communication as ServoComModule
from ServoProjectModules.Communication import CommunicationError


dt = 0.018

def createServoManager(port):
    def createServosFunction():
        simCom = ServoComModule.SimulateCommunication()
        if port != '':
            com = ServoComModule.SerialCommunication(port)
        else:
            print('Simulation mode active')
            com = simCom

        servoArray = []

        newServo = ServoComModule.DCServoCommunicator(1, com)
        newServo.setOffsetAndScaling(360.0 / 4096.0, -110.0, 0)
        newServo.setControlSpeed(32)
        newServo.setBacklashControlSpeed(6, 180.0, 0.00)
        servoArray.append(newServo)

        newServo = ServoComModule.DCServoCommunicator(2, com)
        newServo.setOffsetAndScaling(180.0 / 1900.0, 0.0, 0.0)
        servoArray.append(newServo)

        return servoArray

    servoManager = ServoComModule.ServoManager(cycleTime=dt, initFunction=createServosFunction)

    return servoManager

def playTrajectory(servoManager, trajectory):
    doneRunning = False
    index = 0

    def sendCommandHandlerFunction(cycleTime, servoManager):
        nonlocal index
        nonlocal doneRunning

        if index == 0:
            pos = trajectory[0]
            vel = [0.0] * len(pos)
        elif index < len(trajectory) - 1:
            pos = trajectory[index]
            vel = (np.array(trajectory[index + 1]) - np.array(trajectory[index - 1])) / 2.0 / cycleTime
        else:
            pos = trajectory[-1]
            vel = [0.0] * len(pos)

        for i, servo in enumerate(servoManager.servoArray):
            servo.setReference(pos[i], vel[i], 0.0)

        index += 1
        if index == len(trajectory):
            servoManager.removeHandlerFunctions()
            doneRunning = True

    def readResultHandlerFunction(cycleTime, servoManager):
        pos = servoManager.getPosition()
        print(f'{pos[0] = :0.4f}, {pos[1] = :0.4f}', end='\r')

    def errorHandlerFunction(inputException):
        try:
            raise inputException
        except CommunicationError as e:
            if e.code == CommunicationError.ErrorCode.COULD_NOT_SEND:
                raise e

            print('exception triggered: restarting communication...')
            servoManager.shutdown()
            servoManager.start()

    servoManager.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction, errorHandlerFunction)

    while not doneRunning:
        if not servoManager.isAlive():
            break
        time.sleep(0.1)

def addLinearMove(trajectory, endPos, duration):
    startPos = np.array(trajectory[-1])
    endPos = np.array(endPos)

    steps = int(round(duration / dt))
    for i in range(1, steps + 1):
        t = i / steps
        trajectory.append(endPos * t + startPos * (1 - t))

    return trajectory

def addSmoothMove(trajectory, endPos, duration):
    startPos = np.array(trajectory[-1])
    endPos = np.array(endPos)

    steps = int(round(duration / dt))
    for i in range(1, steps + 1):
        t = i / steps
        t = (1.0 - math.cos(pi * t)) / 2.0
        trajectory.append(endPos * t + startPos * (1 - t))

    return trajectory

def addWait(trajectory, duration):
    startPos = np.array(trajectory[-1])

    steps = int(round(duration / dt))
    for _ in range(1, steps + 1):
        trajectory.append(startPos)

    return trajectory

def main():
    port = '' # '/dev/ttyACM0'
    with createServoManager(port) as servoManager:
        trajectory = []
        trajectory.append(servoManager.getPosition())

        trajectory = addSmoothMove(trajectory, [0.0, 0.0], 1.0)
        trajectory = addWait(trajectory, 10.0)

        for _ in range(0, 3):
            # trajectory = addSmoothMove(trajectory, [2.0, 2.0], 0.2)
            trajectory = addSmoothMove(trajectory, [-10.0, -10.0], 1.0)
            trajectory = addSmoothMove(trajectory, [0.0, 0.0], 3.0)
            trajectory = addWait(trajectory, 3.0)
            # trajectory = addSmoothMove(trajectory, [-2.0, -2.0], 0.2)
            trajectory = addSmoothMove(trajectory, [10.0, 10.0], 1.0)
            trajectory = addSmoothMove(trajectory, [0.0, 0.0], 3.0)
            trajectory = addWait(trajectory, 3.0)

        trajectory = addWait(trajectory, 7.0)

        playTrajectory(servoManager, trajectory)

if __name__ == '__main__':
    main()
