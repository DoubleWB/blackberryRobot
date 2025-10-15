#!/usr/bin/env python
# license removed for brevity
import blackberry_controller_v2 as bb_ctrl
import robot as rob
import time

if __name__ == '__main__':
    #Start robot connection and reset robot
#    bb = bb_ctrl.DynamixelComms(useHardware=False)
#    bb.setJointAngles([0, 0, 0, 0, 0, 0])

    print('=====================================================================')

    bb2 = rob.Blackberry(useHardware=True, startWithTorque=True)
    #bb2.setJointAngles([0, 0, 0, 0, 0, 0]) #Check!
    #bb2.setJointAngles([1.507, -1.507, 0.785, 1.507, -1.507, 1.507]) #Check!
    #bb2.setJointAngles([-1.507, -0.785, 1.507, -1.507, 1.507, -1.507]) #Check!

    #Tuning Testing
    #Gripper - default

    #Joint 6 (WristRot)
    # bb2.setJointAngle(5, -1.571)
    # time.sleep(3)
    # bb2.setJointAngle(5, 1.571)
    # time.sleep(3)
    # bb2.setJointAngle(5, 0)
    # time.sleep(2)
    # bb2.setJointAngle(5, -0.785)
    # time.sleep(2)
    # bb2.setJointAngle(5, -1.571)
    # time.sleep(5)

    # #Joint 4 (WristFlex)
    # bb2.setJointAngle(4, -1.571)
    # time.sleep(3)
    # bb2.setJointAngle(4, 0)
    # time.sleep(2)
    # bb2.setJointAngle(4, -0.785)
    # time.sleep(2)
    # bb2.setJointAngle(4, -1.571)
    # time.sleep(5)

    # #Joint 3 (ElbowRot)
    # bb2.setJointAngle(3, 1.571)
    # time.sleep(3)
    # bb2.setJointAngle(3, -1.571)
    # time.sleep(2)
    # bb2.setJointAngle(3, 0)
    # time.sleep(2)
    # bb2.setJointAngle(3, 0.785)
    # time.sleep(2)
    # bb2.setJointAngle(3, 1.571)
    # time.sleep(5)

    #Joint 2 (Elbow)
    bb2.setJointAngle(2, 1.507)
    time.sleep(3)
    bb2.setJointAngle(2, 1.047)
    time.sleep(2)
    bb2.setJointAngle(2, 0.323)
    time.sleep(2)
    bb2.setJointAngle(2, 1.507)
    time.sleep(5)

    #Joint 1 (Shoulder)
    bb2.setJointAngle(1, -1.507)
    time.sleep(3)
    bb2.setJointAngle(1, -1.047)
    time.sleep(2)
    bb2.setJointAngle(1, -0.323)
    time.sleep(2)
    bb2.setJointAngle(1, -1.507)
    time.sleep(5)

    # #Joint 0 (ShoulderRot)
    # bb2.setJointAngle(0, -1.571)
    # time.sleep(3)
    # bb2.setJointAngle(0, 1.571)
    # time.sleep(3)
    # bb2.setJointAngle(0, 0)
    # time.sleep(2)
    # bb2.setJointAngle(0, -0.785)
    # time.sleep(2)
    # bb2.setJointAngle(0, -1.571)
    # time.sleep(5)

    #print(bb2.getEE([0, 0, 0, 0, 0, 0]))

    bb2.toggleActivate(False)

    print('test done')