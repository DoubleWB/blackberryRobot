import motor
import util
import math
import time
import random
from dynamixel_sdk import COMM_SUCCESS

class Joint:
    #============================= Joint Constants =====================================
    
    _homing_load_threshold = 200 #10%
    _homing_advance_rate = 25 #Starting careful
    _homing_physical_limit = -2.02458193

    #============================= Joint Construction ==================================
    def __init__(self, qID, transform, jointLimits, motorParameters, portHandler, packetHandler, groupBulkWrite, groupSyncReadPos, groupSyncReadLoad, errorCallback, useHardware):
        """Initialize a joint for controlling dynamixel motors as part of a kinematic chain"""
        
        self._qID = qID
        self._packetHandler = packetHandler
        self._groupBulkWrite = groupBulkWrite
        self._groupSyncReadPos = groupSyncReadPos
        self._groupSyncReadLoad = groupSyncReadLoad
        self._errorCallback = errorCallback
        #Initialize homogenous transform from the base to the end of this joint as a function of theta
        self._transform = transform
        self._joint_limits = jointLimits
        self._motors = []
        for _, paramSet in enumerate(motorParameters):
            if useHardware:
                self._motors.append(motor.DynamixelMotor(paramSet, portHandler, packetHandler, groupBulkWrite, groupSyncReadPos, groupSyncReadLoad, errorCallback))

    #============================= Helpers =============================================
    def withinLimits(self, theta):
        """Return True if the given theta is within the limits of this joint"""
        return self._joint_limits[0] <= theta and theta <= self._joint_limits[1]
    
    def enforceLimits(self, theta):
        """Return the given theta bounded by the limits of this joint"""
        return max(self._joint_limits[0], min(theta, self._joint_limits[1]))
    
    def getRandom(self):
        """Returns a random value in radians within the limits of this joint"""
        range = self._joint_limits[1] - self._joint_limits[0]
        return self._joint_limits[0] + random.random() * range;
    
    def getJointMovementStatus(self):
        """Returns the finished moving and under load threshold statuses of a this joint"""
        allMoving = True
        allUnderLoadThreshold = True
        for _, motor in enumerate(self._motors):
            motorMoving, motorUnderLoadThreshold = motor.getMotorMovementStatus()
            allMoving = allMoving and motorMoving
            allUnderLoadThreshold = allUnderLoadThreshold and motorUnderLoadThreshold
        return allMoving, allUnderLoadThreshold
    
    def fetchAndGetJointMovementStatus(self):
        """Refreshes and returns the finished moving and under load threshold statuses of a this joint"""
        dxl_comm_result = self._groupSyncReadPos.txRxPacket()
        readSuccess = util.handleCommResponse(dxl_comm_result, 'Joint {} status (position) read'.format(self._qID), self._packetHandler)
        dxl_comm_result = self._groupSyncReadLoad.txRxPacket()
        readSuccess = readSuccess and util.handleCommResponse(dxl_comm_result, 'Joint {} status (load) read'.format(self._qID), self._packetHandler)
        if readSuccess:
            return self.getJointMovementStatus()
    
    def getTransform(self):
        """Return the symbolic transform from the base of this joint to the next"""
        return self._transform
    
    def toggleActivate(self, activate):
        """Toggles the activation (torque) state of this joint"""
        for _, motor in enumerate(self._motors):
            motor.toggleTorque(activate)

    def homeGearedJoint(self, convex):
        """Dynamically aligns and searches for the zero position of a dual controlled geared joint"""
        if (len(self._motors)) != 2:
            print("Cannot home - not using geared joint.")
            return
        
        print("Homing geared joint - steer clear!")

        #======= Step 1 - move both motors such that their gear teeth are aligned. =======
        motor1 = self._motors[0]
        motor2 = self._motors[1]
        motor1pos = motor1.readMotorPos()
        motor2pos = motor2.readMotorPos()
        motor1Alignment = util.getGearToothAlignmentFromPos(motor1pos)
        motor2Alignment = util.getGearToothAlignmentFromPos(motor2pos)
        #Align to whichever gear tooth is more engaged (closer to 1)
        print("Aligning {}".format(self._qID))
        if (motor1Alignment > motor2Alignment):
            motor2pos = util.getClosestGearToothAlignmentPos(motor1Alignment, motor2pos)
            motor2.safeSendMotorPosition(motor2pos)
        else:
            motor1pos = util.getClosestGearToothAlignmentPos(motor2Alignment, motor1pos)
            motor1.safeSendMotorPosition(motor1pos)

        #======= Step 2 - move until joint limit =======
        #Move both motors in small increments in the negative direction until load indicates we've hit a physical joint limit at xxx radians
        motor1load = motor1.readMotorLoad()
        motor2load = motor2.readMotorLoad()
        motor1rate = motor1.getRadsPerPos()
        motor2rate = motor2.getRadsPerPos()
        motor1dir = int(math.copysign(1, motor1rate))
        motor2dir = int(math.copysign(1, motor2rate))

        print("Calibrating {}".format(self._qID))
        while abs(motor1load) < self._homing_load_threshold and abs(motor2load) < self._homing_load_threshold:
            #advance
            #we subtract the delta because we want to home in the negative direction
            motor1pos = motor1pos - (motor1dir * self._homing_advance_rate)
            motor1.sendMotorPosition(motor1pos)
            motor2pos = motor2pos - (motor2dir * self._homing_advance_rate)
            motor2.sendMotorPosition(motor2pos)
            #Update Loads
            motor1load = motor1.readMotorLoad()
            motor2load = motor2.readMotorLoad()
        
        #Calculate the zero points from the physical limit.
        motor1diff = self._homing_physical_limit / motor1rate
        motor1.setZeroPos(motor1pos - motor1diff)
        motor2diff = self._homing_physical_limit / motor2rate
        motor2.setZeroPos(motor2pos - motor2diff)
        
        # #======= Step 3 - reset homed joint to 0 radians =======
        print("{} calibrated!".format(self._qID))
        if convex:
            self.sendToTheta(-1.20)
            self.sendToTheta(-0.623599)
        else:
            self.sendToTheta(-1.20)
            self.sendToTheta(0)
            self.sendToTheta(0.623599)

    #============================= Joint Angle Control =======================================
    def addPendingTheta(self, theta):
        """Prepares a command to move this joint to the given theta, to be fired off in bulk"""
        if self.withinLimits(theta):
            for _, motor in enumerate(self._motors):
                newPos = motor.posFromRad(theta)
                motor.addMotorPosition(newPos)
                print('Setting {} to angle {}'.format(self._qID, theta))
        else:
            print('Cannot set joint {} to out of bounds theta: {}'.format(self._qID, theta))

    def sendToTheta(self, theta):
        """Sends a command to move this joint to the given theta"""
        if self.withinLimits(theta):
            #Add all positions
            for _, motor in enumerate(self._motors):
                newPos = motor.posFromRad(theta)
                motor.addMotorPosition(newPos)
                print('Setting {} to angle {}'.format(self._qID, theta))
            #Send positions
            util.sendAndClearBulkWrite('Sending Joint Command for: {}'.format(self._qID), self._groupBulkWrite, self._packetHandler)
            #Monitor
            jointMoving = True
            jointUnderLoadThresh = True
            while jointMoving:
                if not jointUnderLoadThresh:
                    self._errorCallback.__call__()
                    return
                time.sleep(util.POLLING_DELAY)
                jointMoving, jointUnderLoadThresh = self.fetchAndGetJointMovementStatus()
        else:
            print('Cannot send joint {} to out of bounds theta: {}'.format(self._qID, theta))

    #============================= Joint Angle Access =======================================
    def readTheta(self):
        """Reads the theta value of this joint. Can return None on comms fail."""
        motorPos = self._motors[0].readMotorPos()
        if motorPos:
            return self._motors[0].radFromPos(motorPos)
        return None

class Gripper:
    #============================= Gripper Construction ==================================
    def __init__(self, gripperLimits, tuningParameters, portHandler, packetHandler, groupBulkWrite, groupSyncReadPos, groupSyncReadLoad, errorCallback, useHardware):
        if useHardware:
            self._motor = motor.DynamixelMotor(tuningParameters, portHandler, packetHandler, groupBulkWrite, groupSyncReadPos, groupSyncReadLoad, errorCallback)
        self._gripper_lims = gripperLimits

    #============================= Gripper Helpers ==================================
    def withinLimits(self, pos):
        """Return True if the given theta is within the limits of this gripper"""
        return self._gripper_lims[0] <= pos and pos <= self._gripper_lims[1]
    
    def toggleActivate(self, activate):
        """Toggles the activation (torque) state of this gripper"""
        self._motor.toggleTorque(activate)
    
    def getGripperRange(self):
        """Calculates the range of valid gripper motor position given the limits of this gripper"""
        return self._gripper_lims[1] - self._gripper_lims[0]

    def gripPosToMotorPos(self, gripVal):
        """Calculates the gripper motor position given the desired gripper open/close value"""
        return self._gripper_lims[0] + (gripVal * self.getGripperRange())

    #============================= Gripper Control ==================================
    def addGripperPostion(self, gripVal):
        """Adds a new position calculated from the given gripper open/close percentage"""
        gripMotorPos = self.gripPosToMotorPos(gripVal)
        if self.withinLimits(gripMotorPos):
            self._motor.addMotorPosition(gripMotorPos)
        else:
            print('Cannot set gripper to out of bounds theta: {}'.format(gripMotorPos))

    def sendGripperPostion(self, gripVal):
        """Adds and sends a new position calculated from the given gripper open/close percentage"""
        gripMotorPos = self.gripPosToMotorPos(gripVal)
        if self.withinLimits(gripMotorPos):
            self._motor.safeSendMotorPosition(gripMotorPos)
        else:
            print('Cannot send gripper to out of bounds theta: {}'.format(gripMotorPos))

    #============================= Gripper Access ==================================
    def readGripperPosition(self):
        gripMotorPosition = self._motor.readMotorPos()
        return (gripMotorPosition - self._gripper_lims[0]) / self.getGripperRange()
    
