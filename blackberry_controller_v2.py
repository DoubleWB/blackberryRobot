#!/usr/bin/env python
# license removed for brevity
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import random
import numpy as np
import time
from sympy import sin, cos, Matrix, symbols

# Dynamixel constants for X_SERIES
ADDR_POSITION_P             = 84
ADDR_POSITION_I             = 82
ADDR_POSITION_D             = 80
LEN_POSITION_PID            = 2
ADDR_FF_ACC_GAIN            = 88    # Feedforward second gain
ADDR_FF_VEL_GAIN            = 90    # Feedforward first gain
LEN_FF_GAIN                 = 2
ADDR_TORQUE_ENABLE          = 64
ADDR_LED_RED                = 65
LEN_LED_RED                 = 1
ADDR_GOAL_POSITION          = 116
LEN_GOAL_POSITION           = 4
ADDR_PRESENT_POSITION       = 132
LEN_PRESENT_POSITION        = 4
BAUDRATE                    = 57600
PROTOCOL_VERSION            = 2.0
TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque

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

    #Current joint angles of the robot
    _q_curr = [0, 0, 0, 0, 0, 0]
    #Array of tuples representing the [min, max] limits for each joint
    _q_limits = [(-3.0542, 3.0542), (-1.5708, 1.5708), (-2.0944, 1.5708), (-6.2832, 6.2832), (-2.0071, 1.7453), (-4.7124, 4.7124)]

    #Current gripper position
    _grip_curr = 0.0
    #Gripper limit range ([0, 1] for close/open)
    _grip_lim = (0.0, 1.0)

    def __init__(self, useHardware = False):
        """Initialize the blackberry robot model, optionally connecting to hardware"""
        print('Blackberry robot initializing...')
        #TODO!!!!!!!!!!!!!!!!!!!!!

class DynamixelComms:
    """Class for handling motor configuration and communication with Dynamixel motors"""
    #Array representing whether a joint is controlled by multiple motors
    _q_paired = [False, True, True, False, False, False]
    #Array representing whether a joint reversed or not 
    _q_rev = [False, True, True, False, False, False]
    #Array representing the zero position for each joint 
    _q_zeroes = [2048, 2048, 2048, 0, 2048, 2048]
    #Array representing tuples of (Kp, Ki, Kd, Kg, Kv) [PID controller parameters, followed by 2nd and first feedforward controller parameters] for each joint
    #TODO: Consider replacing with dataclass storage
    _q_tuning = [(0, 0, 0, 0, 0), (0, 0, 0, 0, 0), (0, 0, 0, 0, 0), (0, 0, 0, 0, 0), (0, 0, 0, 0, 0), (0, 0, 0, 0, 0)]
    #Number of expected joint angles
    _q_read_size = 6
    #Max position on a dynamixel before looping back to 0
    _motor_max = 4095
    #Gripper position range expressed as a tuple
    _gripper_lims = (2600, 3900)
    #Hardcoded index for the gripper so it can be controlled independently of the dynamically allocated dIds
    _gripper_id = 9
    #(Kp, Ki, Kd, Kg, Kv) Tuple for gripper configuration
    _gripper_tuning = (0, 0, 0, 0, 0)
    #Distance from the goal position a motor may be for the angle to be considered achieved (~5 Degrees)
    _epsilon = 0.0873

    #============================= DynamixelComms Construction ==================================
    def __init__(self, useHardware = True, usbPort = '/dev/ttyUSB0'):
        """Initialize a connection to the robot with inverse kinematic control"""

        # Initialize Handlers
        self.portHandler = PortHandler(usbPort)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Initialize GroupBulk
        self.groupBulkWrite = GroupBulkWrite(self.portHandler, self.packetHandler)
        self.groupBulkRead = GroupBulkRead(self.portHandler, self.packetHandler)

        if useHardware:
            # Open port
            if self.portHandler.openPort():
                print('USB port opened successfully')
            else:
                print('USB port opening failed, terminating')
                quit()

            # Set port baudrate
            if self.portHandler.setBaudRate(BAUDRATE):
                print('Baud rate set successfully')
            else:
                print('Baud rate setting failed, terminating')
                quit()

            # Setup readers, configure PID
            self.setupMotors()

    #============================= Comm Utilities ==========================================
    def sendAndClearBulkWrite(self, command_string):
        """Send a groupBulkWrite pending packet of params, and handle logging the result with the given command string before clearing the groupBulkWrite"""
        dxl_comm_result = self.groupBulkWrite.txPacket()
        self.handleCommResponse(dxl_comm_result, command_string)
        self.groupBulkWrite.clearParam()

    def handleCommResponse(self, comm_result, command_string, verbose = False):
        """Handle the repeated logic of checking the result of a dynamixel communication, and printing success/failure with the given command string. Also returns True when the communication was successful."""
        if comm_result != COMM_SUCCESS:
            print('Dynamixel command failure: {} | {}'.format(command_string, self.packetHandler.getTxRxResult(comm_result)))
        elif verbose:
            print('Dynamixel command success: {}'.format(command_string))
        return comm_result == COMM_SUCCESS
    
    def configureMotorReads(self, dID):
        """Initialize the groupBulkRead with all the parameters we want to read from the given dID"""
        dxl_addparam_result = self.groupBulkRead.addParam(dID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        if dxl_addparam_result != True:
            print('Read param setting failed for dynamixel {}'.format(dID))
        else:
            print('Read params set for dynamixel {}'.format(dID))
    
    #============================= Single Motor Utilities ==================================
    def toggleTorque(self, dID, enable):
        """Toggle the torque for a single motor"""
        toggleParam = TORQUE_ENABLE if enable else TORQUE_DISABLE
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dID, ADDR_TORQUE_ENABLE, toggleParam)
        self.handleCommResponse(dxl_comm_result, 'Motor {} torque toggled {}'.format(dID, toggleParam))
        #Additional error handling potential just for single byte writes
        if dxl_error != 0:
            print('{}'.format(self.packetHandler.getRxPacketError(dxl_error)))

    def setPositionPID(self, dID, p, i, d):
        """Sets the PID position parameters for a single motor"""
        #P
        param_position_p = [DXL_LOBYTE(p), DXL_HIBYTE(p)]
        self.groupBulkWrite.addParam(dID, ADDR_POSITION_P, LEN_POSITION_PID, param_position_p)
        #I
        param_position_i = [DXL_LOBYTE(i), DXL_HIBYTE(i)]
        self.groupBulkWrite.addParam(dID, ADDR_POSITION_I, LEN_POSITION_PID, param_position_i)
        #D
        param_position_d = [DXL_LOBYTE(d), DXL_HIBYTE(d)]
        self.groupBulkWrite.addParam(dID, ADDR_POSITION_D, LEN_POSITION_PID, param_position_d)

        self.sendAndClearBulkWrite('Motor {} PID set to: ({}, {}, {})'.format(dID, p, i, d))

    def setPositionFF(self, dID, ff2, ff1):
        """Sets the feedforward gain parameters for a single motor"""
        #FF2
        param_ff_acc = [DXL_LOBYTE(ff2), DXL_HIBYTE(ff1)]
        self.groupBulkWrite.addParam(dID, ADDR_FF_ACC_GAIN, LEN_FF_GAIN, param_ff_acc)
        #FF1
        param_ff_vel = [DXL_LOBYTE(ff1), DXL_HIBYTE(ff1)]
        self.groupBulkWrite.addParam(dID, ADDR_FF_VEL_GAIN, LEN_FF_GAIN, param_ff_vel)

        self.sendAndClearBulkWrite('Motor {} FF set to: ({}, {})'.format(dID, ff2, ff1))

    def tuneMotor(self, dID, tuningParams):
        """Sets all of the parameters for the position based setpoint movement of the given motor and a (Kp, Ki, Kd, Kg, Kv) tuple"""
        p_param = tuningParams[0]
        i_param = tuningParams[1]
        d_param = tuningParams[2]
        ff_acc_param = tuningParams[3]
        ff_vel_param = tuningParams[4]
        self.setPositionPID(dID, p_param, i_param, d_param)
        self.setPositionFF(dID, ff_vel_param, ff_acc_param)

    def addMotorCommand(self, dID, motorPos):
        """Adds a new position to the bulk writer for a single motor. Does not clear previous queued params"""
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(motorPos)), DXL_HIBYTE(DXL_LOWORD(motorPos)), DXL_LOBYTE(DXL_HIWORD(motorPos)), DXL_HIBYTE(DXL_HIWORD(motorPos))]
        self.groupBulkWrite.addParam(dID, ADDR_GOAL_POSITION, LEN_GOAL_POSITION, param_goal_position)
    
    def sendMotorCommand(self, dID, motorPos):
        """Sends a new position for a single motor"""
        self.addMotorCommand(dID, motorPos)
        self.sendAndClearBulkWrite('Motor {} position set to: {}'.format(dID, motorPos))

    def readMotorPos(self, dID):
        """Returns the position value read for a single motor. Can return None on comms fail."""
        dxl_comm_result = self.groupBulkRead.txRxPacket()
        readSuccess = self.handleCommResponse(dxl_comm_result, 'Motor {} position read'.format(dID))
        if readSuccess:
            dxl_getdata_result = self.groupBulkRead.isAvailable(dID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if dxl_getdata_result == True:
                return self.groupBulkRead.getData(dID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                
        print('Motor {} position fetch failed'.format(dID))
        return None
    
    def addMotorsByAngle(self, theta, qInd):
        """Adds a new position calculated from the given theta to the motor(s) associated with the given qInd joint"""
        #Figure out the correct motor id(s) to set for the given angle, starting from the first motor id 
        dID = 1
        for i in range(qInd):
            #increment dID and proceed
            dID += 2 if self._q_paired[i] else 1
        print('Setting {} to {}'.format(dID, self._q_zeroes[qInd] + (round(theta/(2 * np.pi) * self._motor_max) * (-1 if self._q_rev[qInd] else 1))))
        self.addMotorCommand(dID, self._q_zeroes[qInd] + (round(theta/(2 * np.pi) * self._motor_max) * (-1 if self._q_rev[qInd] else 1)))
        if self._q_paired[qInd]:
            #If the given q index is a paired joint, add one more command for the next motor in the reverse direction
            print('Setting {} to {}'.format(dID, self._q_zeroes[qInd] + (round(theta/(2 * np.pi) * self._motor_max) * (-1 if self._q_rev[qInd] else 1))))
            self.addMotorCommand(dID + 1, self._q_zeroes[qInd] + (round(theta/(2 * np.pi) * self._motor_max) * (1 if self._q_rev[qInd] else -1)))

    def sendMotorsByAngle(self, theta, qInd):
        """Adds and sends a new position calculated from the given theta to the motor(s) associated with the given qInd joint"""
        self.addMotorsByAngle(theta, qInd)
        self.sendAndClearBulkWrite('Joint {} angle set to: {}'.format(qInd, theta))

    def readJointAngle(self, qInd):
        """Reads the current position calculated as an angle in radians from the motor(s) associated with the given qInd joint. Can return None on comms fail."""
        #Figure out the correct motor id(s) to read for the given angle, starting from the first motor id 
        dID = 1
        for i in range(qInd):
            #increment dID and proceed
            dID += 2 if self._q_paired[i] else 1
        motorPos = self.readMotorPos(dID)
        if motorPos:
            return ((motorPos - self._q_zeroes[qInd]) * 2 * np.pi)/(self._motor_max * (1 if self._q_rev[qInd] else -1))
        return None

    def addGripperPostion(self, gripVal):
        """Adds a new position calculated from the given gripper open/close percentage"""
        gripRange = self._gripper_lims[1] - self._gripper_lims[0]
        gripPosDesired = self._gripper_lims[0] + (gripVal * gripRange)
        self.addMotorCommand(self._gripper_id, gripPosDesired)

    def sendGripperPostion(self, gripVal):
        """Adds and sends a new position calculated from the given gripper open/close percentage"""
        self.addGripperPostion(gripVal)
        self.sendAndClearBulkWrite('Gripper position set to: {}'.format(gripVal))

    #============================= Robot Utilities ==================================
    def setupMotors(self, startWithTorque = True):
        """Initializes read params, configures the PID and feedforward params, and optionally toggles torque on for all motors"""
        #Sets the joint by joint tuning values
        dID = 1
        for q in range(self._q_read_size):
            self.tuneMotor(dID, self._q_tuning[q])
            self.toggleTorque(dID, startWithTorque)
            dID += 1
            if self._q_paired[q]:
                self.tuneMotor(dID, self._q_tuning[q])
                self.toggleTorque(dID, startWithTorque)
                dID += 1
        #Sets the gripper tuning values
        g_p_param = self._gripper_tuning[0]
        g_i_param = self._gripper_tuning[1]
        g_d_param = self._gripper_tuning[2]
        g_ff_acc_param = self._gripper_tuning[3]
        g_ff_vel_param = self._gripper_tuning[4]
        self.setPositionPID(self._gripper_id, g_p_param, g_i_param, g_d_param)
        self.setPositionFF(self._gripper_id, g_ff_vel_param, g_ff_acc_param)
        self.toggleTorque(self._gripper_id, startWithTorque)

    def setJointAngles(self, q):
        """Adds and sends commands to all motors given an array of joint angles"""
        for ind, ang in enumerate(q):
            print("setJointAngles {} to {}".format(ind, ang))
            self.addMotorsByAngle(ang, ind)
        #self.sendAndClearBulkWrite('Joint angles set to: {}'.format(q))

    def readAllJointAngles(self):
        """Adds and sends commands to all motors given an array of joint angles"""
        q = []
        for i in range(self._q_read_size):
            q.append(self.readJointAngle(i))
        return q
    
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