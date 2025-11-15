from dynamixel_sdk import * 
from sympy import sin, cos, asin, atan2, Matrix, pi, eye, symbols, lambdify
import joint
import math
import util
import numpy as np

# Dynamixel constants for X_SERIES
BAUDRATE                    = 57600
PROTOCOL_VERSION            = 2.0
ADDR_PRESENT_POSITION       = 132
LEN_PRESENT_POSITION        = 4
ADDR_PRESENT_LOAD           = 126
LEN_PRESENT_LOAD            = 2

class Blackberry:
    """Class representing the blackberry robot and its joints and transforms"""
    #============================= Class members ==================================

    # Joint Descriptions
    # 0 = Shoulder Rotation
    # 1 = Shoulder Flexion
    # 2 = Elbow Flexion
    # 3 = Forearm Rotation
    # 4 = Wrist Flexion
    # 5 = Wrist Rotation
    # 6 = Gripper Position

    # All joint values are internally represented in radians
    # Zeroed angle values represent the robot facing forward and pointing straight up

    # ========= Kinematic Members =========
    _solve_attempts = 20
    _max_iterations = 250
    _error_epsilon = 0.001
    _learning_rate = 0.05
    _decay_per_iteration = .999

    #Current joint angles of the robot
    _q = [0, 0, 0, 0, 0, 0]
    #Symbols for the thetas of each joint to be used in the accompanying 
    _q0, _q1, _q2, _q3, _q4, _q5 = symbols('_q0 _q1 _q2 _q3 _q4 _q5')
    _inputSymbols = [_q0, _q1, _q2, _q3, _q4, _q5]
    #Linkage and sub linkage lengths (meters)
    _l0 = 0.135
    _l1z = 0.185
    _l1x = 0.080
    _l2 = 0.115
    _l3 = 0.078
    _l4 = 0.065
    _l5 = 0.105
    #Per joint transforms (written in symbolic code, so inconvenient to extract from a config file - hardcoding instead)
    _q_trans = [
                #T01
                Matrix([[cos(_q0),  -sin(_q0), 0,         0],
                        [sin(_q0),  cos(_q0),  0,         0],
                        [0,         0,         1,         _l0],
                        [0,         0,         0,         1]]),
                #T12
                Matrix([[cos(_q1),  0,         sin(_q1),  (_l1z * sin(_q1)) + (_l1x * sin(pi/2 - _q1))],
                        [0,         1,         0,         0],
                        [-sin(_q1), 0,         cos(_q1),  (_l1z * cos(_q1)) - (_l1x * cos(pi/2 - _q1))],
                        [0,         0,         0,         1]]),
                #T23
                Matrix([[cos(_q2),  0,         sin(_q2),  _l2 * cos(_q2)],
                        [0,         1,         0,         0],
                        [-sin(_q2), 0,         cos(_q2),  -(_l2 * sin(_q2))],
                        [0,         0,         0,         1]]),
                #T34
                Matrix([[1,         0,         0,         _l3],
                        [0,         cos(_q3),  -sin(_q3), 0],
                        [0,         sin(_q3),  cos(_q3),  0],
                        [0,         0,         0,         1]]),
                #T45
                Matrix([[cos(_q4),  0,         sin(_q4),  _l4 * cos(_q4)],
                        [0,         1,         0,         0],
                        [-sin(_q4), 0,         cos(_q4),  -(_l4 * sin(_q4))],
                        [0,         0,         0,         1]]),
                #T56
                Matrix([[1,         0,         0,         _l5],
                        [0,         cos(_q5),  -sin(_q5), 0],
                        [0,         sin(_q5),  cos(_q5),  0],
                        [0,         0,         0,         1]])]

    def __init__(self, useHardware = True, usbPort = '/dev/ttyUSB0', startWithTorque = True):
        """Initialize a connection to the robot with inverse kinematic control"""

        # Initialize Handlers
        self._portHandler = PortHandler(usbPort)
        self._packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Initialize Writers and Readers
        self._groupBulkWrite = GroupBulkWrite(self._portHandler, self._packetHandler)
        self._groupSyncReadPos = GroupSyncRead(self._portHandler, self._packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        self._groupSyncReadLoad = GroupSyncRead(self._portHandler, self._packetHandler, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD)

        if useHardware:
            # Open port
            if self._portHandler.openPort():
                print('USB port opened successfully')
            else:
                print('USB port opening failed, terminating')
                quit()

            # Set port baudrate
            if self._portHandler.setBaudRate(BAUDRATE):
                print('Baud rate set successfully')
            else:
                print('Baud rate setting failed, terminating')
                quit()

        config = util.getArmConfiguration()

        #Initialize all joints from config
        jointConfigs = config['arm']['joints']
        self._joints = []
        self._T06 = eye(4)
        for i, jointConfig in enumerate(jointConfigs):
            nextJoint = joint.Joint(i, self._q_trans[i], jointConfig['limits'], jointConfig['motors'], self._portHandler, self._packetHandler, self._groupBulkWrite, self._groupSyncReadPos, self._groupSyncReadLoad, self.errorResponse, useHardware)
            if startWithTorque:
                nextJoint.toggleActivate(True)
            self._joints.append(nextJoint)
            self._T06 *= self._q_trans[i]

        #Pre-compute expensive kinematic functions
        self._fkNumeric = lambdify(self._inputSymbols, self.getFK())
        self._jacobianNumeric = lambdify(self._inputSymbols, self.getFK().jacobian(self._inputSymbols))

        #Initialize Gripper
        gripperConfig = config['arm']['gripper']
        self._gripper = joint.Gripper(gripperConfig['limits'], gripperConfig['motor'], self._portHandler, self._packetHandler, self._groupBulkWrite, self._groupSyncReadPos, self._groupSyncReadLoad, self.errorResponse, useHardware)
        if startWithTorque:
            self._gripper.toggleActivate(True)

        #Calibrate Robot
        if useHardware:
            self.calibrationRoutine()
        
    #============================= Arm Helpers ==================================   
    def calibrationRoutine(self):
        """Hard coded calibration sequence to ensure the safe startup of the arm from any previous state"""
        #This startup function serves to combat the fact that the motors' position is reset to values within one rotation upon startup,
        #and joints 1 and 2 rotate multiple times in the full range of the joint.

        #Step 1: for each joint that is limited to 1 rotation, move to a configuration that is safe to not collide while the geared joints are homing.
        self.setJointAngle(5, 0)
        self.setJointAngle(4, np.pi/2)
        self.setJointAngle(3, 0)
        self.setJointAngle(0, np.pi/2)

        #Step 2: Home the two geared joints 
        self.getJoint(2).homeGearedJoint(False)
        self.getJoint(1).homeGearedJoint(True)

    def toggleActivate(self, enableTorque):
        """Toggles the torque activation state for each joint and the gripper in this robot"""
        for _, joint in enumerate(self._joints):
            joint.toggleActivate(enableTorque)
        self._gripper.toggleActivate(enableTorque)

    #============================= Arm Control ==================================
    def setJointAngles(self, q):
        """Adds and sends commands to all motors synchronously given an array of joint angles"""
        #Send an angle to each joint
        for ind, ang in enumerate(q):
            self._joints[ind].addPendingTheta(ang)

        #Write all pending angles
        util.sendAndClearBulkWrite('Sending all joint angles for robot', self._groupBulkWrite, self._packetHandler)
        
        #Monitor for safety and block until the angles are achieved
        anglesAchieved = False
        polls = 0
        #If enough polls elapse to cover the _movement_time_limit and no other issues have occured, we might have trouble reaching this pose for config reasons
        while not anglesAchieved and polls < util.MOVEMENT_TIME_LIMIT/util.POLLING_DELAY:
            anglesAchieved = True
            dxl_comm_result = self._groupSyncReadPos.txRxPacket()
            readSuccess = util.handleCommResponse(dxl_comm_result, 'Robot status (position) read', self._packetHandler)
            dxl_comm_result = self._groupSyncReadLoad.txRxPacket()
            readSuccess = readSuccess and util.handleCommResponse(dxl_comm_result, 'Robot status (load) read', self._packetHandler)
            if not readSuccess:
                self.errorResponse()
                return
            for _, joint in enumerate(self._joints):
                jointMoving, jointUnderLoadThresh = joint.getJointMovementStatus()
                if not jointUnderLoadThresh:
                    self.errorResponse()
                    return
                #we can't break early because we still want to check the load of each joint
                anglesAchieved = anglesAchieved and not jointMoving
            time.sleep(util.POLLING_DELAY)
            polls += 1

    def setIndividualJointAngles(self, q):
        """Adds and sends commands to each motor one at a time given an array of joint angles"""
        for ind, ang in enumerate(q):
            self._joints[ind].sendToTheta(ang)

    def readAllJointAngles(self):
        """Adds and sends commands to all motors given an array of joint angles"""
        q = []
        for _, joint in enumerate(self._joints):
            q.append(joint.readTheta())
        return q
    
    def getJoint(self, qInd):
        """Returns the joint object if it is present in this robot arm, otherwise None"""
        if qInd >= len(self._joints):
            print('Joint {} out of bounds for this robot')
            return None
        return self._joints[qInd]
    
    def setJointAngle(self, qInd, theta):
        """Adds and sends commands to one motor given a joint angle"""
        self.getJoint(qInd).sendToTheta(theta)
    
    def errorResponse(self):
        """Callback function invoked upon any arm joint encountering a run time error"""
        print('ERROR: arm issue detected - attempting graceful shutdown.')
        self.toggleActivate(False)
        quit()

    #============================= Grip Control ==================================
    def setGripper(self, gripVal):
        """Adds and sends a gripper command based on the given gripper percentage"""
        self._gripper.sendGripperPostion(gripVal)

    #============================= Pose Teaching and Control ==================================
    def savePose(self):
        """Runs a routine through which the user can manually pose the robot and save the pose."""
        print('Cutting torque in...')
        delay = 3
        for i in range(delay):
            print('{}...'.format(delay - i))
            time.sleep(1)
        self.toggleActivate(False)
        input('Hit enter to accept pose!')
        self.toggleActivate(True)
        poseQ = self.readAllJointAngles()
        poseName = input('Enter pose nane: ')
        util.savePoseToFile(poseQ, poseName)

    def goToSavedPose(self, poseName):
        """Reads the given pose from the local pose file and moves to that pose if present"""
        q = util.readPoseFromFile(poseName)
        if q:
            self.setJointAngles(q)

    #============================= Kinematics ==================================
    def getFK(self):
        """Return a 1x6 matrix of the functions to calculate [x, y, z, yaw, pitch, roll] given q"""
        #We have to use the YPR angle order as the transformation from T0 to T6 introduces the angles in that order
        return Matrix([[self._T06[0, 3], self._T06[1, 3], self._T06[2, 3],  atan2(self._T06[1, 0], self._T06[0, 0]), asin(-self._T06[2, 0]), atan2(self._T06[2,1], self._T06[2,2])]])

    def getEE(self, q):
        """Get the [x, y, z, yaw, pitch, roll] of the end effector in the base frame given an array of joint angles"""
        #Extract the first row of the 1x6 pose matrix
        return self._fkNumeric(*q)[0]

    def IK(self, goalPose):
        """Return the joint configuration that will achieve the given goal transform, or None if the goal transform is unreachable or at a singularity"""
        print('Running IK')
        #q = self.readAllJointAngles()
        q = [0, 0, 0, 0, 0, 0]
        att = 0
        candidatePoses = {}
        while att < self._solve_attempts:
            print('.')
            it = 0
            err = 99999
            lastErr = err
            bestErr = 99999
            bestQ = []
            while it < self._max_iterations:
                #Calculate error (difference between current pose and goal pose)
                curPose = self.getEE(q)
                dX = util.getErrorVector(goalPose, curPose)
                err = util.getMagnitude(dX)
                #Exit early if the current joint configuration results in an extremely close pose
                if err < self._error_epsilon:
                    return err, q
                #Use gradient descent to generate a closer q
                dQ = (np.linalg.pinv(self._jacobianNumeric(*q)) * dX)
                #Adjust learning rate to decay as we get further in iterations (should it be proportional to our distance to the goal instead?)
                alpha = self._learning_rate * (self._decay_per_iteration ** it)
                dQ *= alpha
                #Break if there's no change in error - we're likely at a local minimum
                if (lastErr == err):
                    break
                if (err < bestErr):
                    bestErr = err
                    bestQ = q
                #Update q, enforcing joint limits
                for i in range(len(q)):
                    q[i] = self.getJoint(i).enforceLimits(float(q[i] + dQ[i]))
                it += 1
                lastErr = err
            candidatePoses[bestErr] = bestQ
            #Generate a new random starting configuration in case it generates a better solution
            newInBoundsQ = []
            for _, joint in enumerate(self._joints):
                newInBoundsQ.append(joint.getRandom())
            q = newInBoundsQ
            att += 1
        lowestError = min(candidatePoses)
        return lowestError, candidatePoses[lowestError]
