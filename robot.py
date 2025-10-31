from dynamixel_sdk import * 
from sympy import sin, cos, Matrix, eye, symbols
import joint
import util

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

    #Current joint angles of the robot
    _q = [0, 0, 0, 0, 0, 0]
    #Symbols for the thetas of each joint to be used in the accompanying 
    _q1, _q2, _q3, _q4, _q5, _q6 = symbols('_q1 _q2 _q3 _q4 _q5 _q6')
    _q_trans = [Matrix([[cos(_q1),  -sin(_q1), 0,         0],
                        [sin(_q1),  cos(_q1),  0,         0],
                        [0,         0,         1,         0.123],
                        [0,         0,         0,         1]]),

                Matrix([[cos(_q2),  0,         sin(_q2),  0.073],
                        [0,         1,         0,         0],
                        [-sin(_q2), 0,         cos(_q2),  0.188],
                        [0,         0,         0,         1]]),

                Matrix([[cos(_q3),  0,         sin(_q3),  0.112],
                        [0,         1,         0,         0],
                        [-sin(_q3), 0,         cos(_q3),  0],
                        [0,         0,         0,         1]]),

                Matrix([[1,         0,         0,         0.080],
                        [0,         cos(_q4),  -sin(_q4), 0],
                        [0,         sin(_q4),  cos(_q4),  0],
                        [0,         0,         0,         1]]),

                Matrix([[cos(_q5),  0,         sin(_q5),  0.065],
                        [0,         1,         0,         0],
                        [-sin(_q5), 0,         cos(_q5),  0],
                        [0,         0,         0,         1]]),

                Matrix([[1,         0,         0,         0.110],
                        [0,         cos(_q6),  -sin(_q6), 0],
                        [0,         sin(_q6),  cos(_q6),  0],
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
        self._FK = eye(4)
        for i, jointConfig in enumerate(jointConfigs):
            if useHardware:
                nextJoint = joint.Joint(i, self._q_trans[i], jointConfig['limits'], jointConfig['motors'], self._portHandler, self._packetHandler, self._groupBulkWrite, self._groupSyncReadPos, self._groupSyncReadLoad, self.errorResponse)
                if startWithTorque:
                    nextJoint.toggleActivate(True)
                self._joints.append(nextJoint)
            self._FK *= self._q_trans[i]

        #Initialize Gripper
        gripperConfig = config['arm']['gripper']
        if useHardware:
            self._gripper = joint.Gripper(gripperConfig['limits'], gripperConfig['motor'], self._portHandler, self._packetHandler, self._groupBulkWrite, self._groupSyncReadPos, self._groupSyncReadLoad, self.errorResponse)
            if startWithTorque:
                self._gripper.toggleActivate(True)

        #Calibrate Robot
        self.calibrationRoutine()
        
    #============================= Arm Helpers ==================================   
    def calibrationRoutine(self):
        """Hard coded calibration sequence to ensure the safe startup of the arm from any previous state"""
        #This startup function serves to combat the fact that the motors' position is reset to values within one rotation upon startup,
        #and joints 1 and 2 rotate multiple times in the full range of the joint.

        #Step 1: for each joint that is limited to 1 rotation, move to a configuration that is safe to not collide while the geared joints are homing.
        self.setJointAngle(5, 0)
        self.setJointAngle(4, 1.5708)
        self.setJointAngle(3, 0)
        self.setJointAngle(0, 0)

        #Step 2: Home the two geared joints 
        self.getJoint(2).homeGearedJoint(False)
        self.getJoint(1).homeGearedJoint(True)

    def toggleActivate(self, enableTorque):
        """Toggles the torque activation state for each joint and the gripper in this robot"""
        for _, joint in enumerate(self._joints):
            joint.toggleActivate(enableTorque)
        self._gripper.toggleActivate(enableTorque)

    def eulerAnglesToRotMatrix(roll, pitch, yaw):
        """Computes the rotation matrix equivalent of the given euler angles"""
        r, p, y = symbols('r p y')
        rollRot =  Matrix([[1,       0,       0],
                           [0,       cos(r),  -sin(r)],
                           [0,       sin(r),  cos(r)]]),

        pitchRot = Matrix([[cos(p),  0,       sin(p)],
                           [0,       1,       0],
                           [-sin(p), 0,       cos(p)]])

        yawRot =   Matrix([[cos(y),  -sin(y), 0],
                           [sin(y),  cos(y),  0],
                           [0,       0,       1]])
        
        fullRot = rollRot * pitchRot * yawRot
        substituteDict = {r: roll,
                          p: pitch,
                          y: yaw}
        return fullRot.evalf(subs=substituteDict)

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
        while not anglesAchieved:
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

    def getEE(self, q):
        """Get the homogeneous transform of the end effector in the base frame given an array of joint angles"""
        substituteDict = {self._q1: q[0],
                          self._q2: q[1],
                          self._q3: q[2],
                          self._q4: q[3],
                          self._q5: q[4],
                          self._q6: q[5]}
        return self._FK.evalf(subs=substituteDict)
    
    def errorResponse(self):
        """Callback function invoked upon any arm joint encountering a run time error"""
        print('ERROR: arm issue detected - attempting graceful shutdown.')
        self.toggleActivate(False)
        quit()

    #============================= Grip Control ==================================
    def setGripper(self, gripVal):
        """Adds and sends a gripper command based on the given gripper percentage"""
        self._gripper.sendGripperPostion(gripVal)