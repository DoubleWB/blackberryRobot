#!/usr/bin/env python
# license removed for brevity
import robot as rob

if __name__ == '__main__':

    bb2 = rob.Blackberry(useHardware=True, startWithTorque=True)
    qCur = bb2.readAllJointAngles()
    print(qCur)
    bb2.setJointAngles([-0.436332, -0.436332, -0.436332, -0.436332, -0.436332, -0.436332])
    bb2.toggleActivate(False)

    print('test done')