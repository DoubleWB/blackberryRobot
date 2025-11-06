#!/usr/bin/env python
# license removed for brevity
import robot as rob
import numpy as np
import math

def inputBasedTest():
    bb2 = rob.Blackberry(useHardware=False, startWithTorque=False)
    np.set_printoptions(suppress=True)
    print(bb2.getEE([1.3, -1.2, 1.6, -1.6, 1.75, 2.4]))
    while True:
        print('Currently at: [x, y, z, yaw, pitch, roll]')
        #print(bb2.getEE(bb2.readAllJointAngles()))
        x = float(input('Enter goal x: '))
        y = float(input('Enter goal y: '))
        z = float(input('Enter goal z: '))
        yaw = float(input('Enter goal yaw: '))
        pitch = float(input('Enter goal pitch: '))
        roll = float(input('Enter goal roll: '))
        q = bb2.IK([x, y, z, yaw, pitch, roll])
        print('Closest Configuration: {}'.format(q))
        print('Would lead to pose: {}'.format(bb2.getEE(q)))
        if input('Move to pose? y/any: ') == 'y':
            bb2.setJointAngles(q)

if __name__ == '__main__':
    inputBasedTest()