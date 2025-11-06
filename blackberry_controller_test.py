#!/usr/bin/env python
# license removed for brevity
import robot as rob
import numpy as np
import math

def inputBasedTest():
    bb2 = rob.Blackberry(useHardware=True, startWithTorque=True)
    np.set_printoptions(suppress=True)
    while True:
        print('Currently at: [x, y, z, yaw, pitch, roll]')
        print(bb2.getEE(bb2.readAllJointAngles()))
        x = float(input('Enter goal x: '))
        y = float(input('Enter goal y: '))
        z = float(input('Enter goal z: '))
        yaw = float(input('Enter goal yaw: '))
        pitch = float(input('Enter goal pitch: '))
        roll = float(input('Enter goal roll: '))
        _, q = bb2.IK([x, y, z, yaw, pitch, roll])
        print('Closest Configuration: {}'.format(q))
        print('Would lead to pose: {}'.format(bb2.getEE(q)))
        if input('Move to pose? y/any: ') == 'y':
            bb2.setJointAngles(q)

def traceCircleTest(xCenter, yCenter, zCenter, radius):
    bb2 = rob.Blackberry(useHardware=True, startWithTorque=True)
    np.set_printoptions(suppress=True)
    theta = 0
    while theta <= 2 * np.pi:
        yMod = radius*np.cos(theta)
        zMod = radius*np.sin(theta)
        err, q= bb2.IK([xCenter, yCenter + yMod, zCenter + zMod, 0, 0, 0])
        print('Closest Configuration: {}'.format(q))
        print('Would lead to pose: {}'.format(bb2.getEE(q)))
        if err <= 0.005: #An error reasonable enough that we can expect no dangerous angle configurations that could lead to self collisions
            bb2.setJointAngles(q)
        theta += np.pi/8
    bb2.toggleActivate(False)


if __name__ == '__main__':
    traceCircleTest(.25, 0, .25, .05)