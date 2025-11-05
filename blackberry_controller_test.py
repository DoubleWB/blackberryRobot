#!/usr/bin/env python
# license removed for brevity
import robot as rob
import numpy as np
import math

if __name__ == '__main__':

    bb2 = rob.Blackberry(useHardware=True, startWithTorque=True)
    np.set_printoptions(suppress=True)
    while True:
        print('Currently at: [x, y, z, y, p, r]')
        print(bb2.getEE(bb2.readAllJointAngles()))
        x = float(input('Enter goal x: '))
        y = float(input('Enter goal y: '))
        z = float(input('Enter goal z: '))
        y = float(input('Enter goal y: '))
        p = float(input('Enter goal p: '))
        r = float(input('Enter goal r: '))
        q = bb2.IK([x, y, z, y, p, r])
        print('Closest Configuration: {}'.format(q))
        print('Would lead to pose: {}'.format(bb2.getEE(q)))
        if input('Move to pose? y/any: ') == 'y':
            bb2.setJointAngles(q)