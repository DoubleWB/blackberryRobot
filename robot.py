from dynamixel_sdk import * 
from sympy import sin, cos, Matrix, eye, symbols
import joint
import util

# Dynamixel constants for X_SERIES
BAUDRATE                    = 57600
PROTOCOL_VERSION            = 2.0

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

        # Initialize GroupBulk
        self._groupBulkWrite = GroupBulkWrite(self._portHandler, self._packetHandler)
        self._groupBulkRead = GroupBulkRead(self._portHandler, self._packetHandler)

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
                nextJoint = joint.Joint(i, self._q_trans[i], jointConfig['limits'], jointConfig['motors'], self._portHandler, self._packetHandler, self._groupBulkWrite, self._groupBulkRead)
                if startWithTorque:
                    nextJoint.toggleActivate(True)
                self._joints.append(nextJoint)
            self._FK *= self._q_trans[i]

        #Initialize Gripper
        gripperConfig = config['arm']['gripper']
        if useHardware:
            self._gripper = joint.Gripper(gripperConfig['limits'], gripperConfig['motor'], self._portHandler, self._packetHandler, self._groupBulkWrite, self._groupBulkRead)
            if startWithTorque:
                self._gripper.toggleActivate(True)

        
        
    #============================= Arm Helpers ==================================   
    def sendPendingCommands(self):
        """Send a groupBulkWrite pending packet of params, and handle logging the result with the given command string before clearing the groupBulkWrite"""
        comm_result = self._groupBulkWrite.txPacket()
        if comm_result != COMM_SUCCESS:
            print('Robot command sending failed: {}'.format(self._packetHandler.getTxRxResult(comm_result)))
        self._groupBulkWrite.clearParam()

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
        """Adds and sends commands to all motors given an array of joint angles"""
        for ind, ang in enumerate(q):
            self._joints[ind].addPendingTheta(ang)
        self.sendPendingCommands()

    def setIndividualJointAngles(self, q):
        """Adds and sends commands to each motor given an array of joint angles"""
        for ind, ang in enumerate(q):
            self._joints[ind].sendToTheta(ang)

    def readAllJointAngles(self):
        """Adds and sends commands to all motors given an array of joint angles"""
        q = []
        for _, joint in enumerate(self._joints):
            q.append(joint.readTheta())
        return q
    
    def setJointAngle(self, qInd, theta):
        """Adds and sends commands to one motors given a joint angles"""
        self._joints[qInd].sendToTheta(theta)
    
    def achievePose(self, q):
        """Command a set of joint angles and wait until the pose is achieved within _epsilon"""
        self.setJointAngles(self, q)
        qPrime = self.readAllJointAngles()
        poseAchieved = False
        while poseAchieved:
            poseAchieved = True
            for i in range(q.len):
                poseAchieved = poseAchieved and abs(q[i] - qPrime[i]) > self._epsilon
                if not poseAchieved:
                    time.sleep(0.1)
                    break

    def getEE(self, q):
        """Get the homogeneous transform of the end effector in the base frame given an array of joint angles"""
        substituteDict = {self._q1: q[0],
                          self._q2: q[1],
                          self._q3: q[2],
                          self._q4: q[3],
                          self._q5: q[4],
                          self._q6: q[5]}
        return self._FK.evalf(subs=substituteDict)

    #============================= Grip Control ==================================
    def setGripper(self, gripVal):
        """Adds and sends a gripper command based on the given gripper percentage"""
        self._gripper.sendGripperPostion(gripVal)
