#!/usr/bin/env python
# license removed for brevity
import robot as rob
import numpy as np
import time
import random
import os
import util
import math
import threading


pause_idle = threading.Event()

def animateIdle(bb):
    """Test routine moving the robot from demo pose to demo pose over random long intervals - intended to be run in a separate thread."""
    minWaitTime = 15
    maxWaitTime = 30 
    idlePoseNames = ['look1', 'look2', 'look3', 'look4', 'look5']
    lastPoseName = ''
    while(not pause_idle.is_set()):
        waitRange = maxWaitTime - minWaitTime
        randomWaitTime = int(minWaitTime + random.random() * waitRange)
        durationWaited = 0
        while(not pause_idle.is_set() and durationWaited < randomWaitTime):
            time.sleep(1)
            durationWaited += 1
        curPoseName = random.choice(idlePoseNames)
        bb.goToSavedPose(curPoseName)
        #Remove the current pose and re-add the last pose to the idle pose options to prevent repeats
        idlePoseNames.remove(curPoseName)
        idlePoseNames.append(lastPoseName)
        lastPoseName = curPoseName
    pause_idle.clear()

def inputBasedTest(bb):
    """Test routine allowing the user to input a goal pose numerically, and decide whether or not to go to the pose"""
    print('Currently at: [x, y, z, yaw, pitch, roll]')
    print(bb.getEE(bb.readAllJointAngles()))
    x = float(input('Enter goal x: '))
    y = float(input('Enter goal y: '))
    z = float(input('Enter goal z: '))
    yaw = float(input('Enter goal yaw: '))
    pitch = float(input('Enter goal pitch: '))
    roll = float(input('Enter goal roll: '))
    _, q = bb.IK([x, y, z, yaw, pitch, roll])
    print('Closest Configuration: {}'.format(q))
    print('Would lead to pose: {}'.format(bb.getEE(q)))
    if input('Move to pose? y/any: ') == 'y':
        bb.setJointAngles(q)

def traceCircleTest(bb, xCenter, yCenter, zCenter, radius):
    """Test routine using IK to trace a circle of a finite number of points in the Y/Z plane"""
    theta = 0
    while theta <= 2 * np.pi:
        yMod = radius*np.cos(theta)
        zMod = radius*np.sin(theta)
        err, q = bb.IK([xCenter, yCenter + yMod, zCenter + zMod, 0, 0, 0])
        print('Closest Configuration: {}'.format(q))
        print('Would lead to pose: {}'.format(bb.getEE(q)))
        if err <= 0.005: #An error reasonable enough that we can expect no dangerous angle configurations that could lead to self collisions
            bb.setJointAngles(q)
        theta += np.pi/8
    bb.toggleActivate(False)

if __name__ == '__main__':
    bb = rob.Blackberry(useHardware=True, startWithTorque=True)
    np.set_printoptions(suppress=True)
    while True:
        idleThread = threading.Thread(target = animateIdle, args = (bb, ))
        idleThread.start()
        print('Blackberry is currently idling, please input one of the following:')
        print('1: Input goal pose!')
        print('2: Teach pose kinesthetically!')
        print('3: Display known pose names!')
        print('4: Go to saved pose by name!')
        print('5: Trace a circle in the Y/Z plane!')
        selection = int(input('Input selection: '))
        pause_idle.set()
        idleThread.join()
        if selection == 1:
            inputBasedTest(bb)
            os.system('cls' if os.name == 'nt' else 'clear')
        elif selection == 2:
            bb.savePose()
            os.system('cls' if os.name == 'nt' else 'clear')
        elif selection == 3:
            util.showPoseNamesFromFile()
        elif selection == 4:
            goalPoseName = input("Input pose name: ")
            bb.goToSavedPose(goalPoseName)
            os.system('cls' if os.name == 'nt' else 'clear')
        elif selection == 5:
            radius = float(input("Input desired circle radius in m: "))
            traceCircleTest(bb, 0.25, 0, 0.25, radius)

