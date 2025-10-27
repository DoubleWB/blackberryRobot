import motor
import numpy as np

class Joint:
    #============================= Joint Constants =====================================
    
    #Distance from the goal position a motor may be for the angle to be considered achieved (~5 Degrees)
    _epsilon = 0.0873

    #============================= Joint Construction ==================================
    def __init__(self, qID, transform, jointLimits, motorParameters, portHandler, packetHandler, groupBulkWrite, groupSyncReadPos, groupSyncReadLoad, errorCallback):
        """Initialize a joint for controlling dynamixel motors as part of a kinematic chain"""
        
        self._qID = qID
        #Initialize homogenous transform from the base to the end of this joint as a function of theta
        self._transform = transform
        self._joint_limits = jointLimits
        self._motors = []
        for _, paramSet in enumerate(motorParameters):
            self._motors.append(motor.DynamixelMotor(paramSet, portHandler, packetHandler, groupBulkWrite, groupSyncReadPos, groupSyncReadLoad, errorCallback))

    #============================= Helpers =============================================
    def withinLimits(self, theta):
        """Return True if the given theta is within the limits of this joint"""
        return self._joint_limits[0] <= theta and theta <= self._joint_limits[1]
    
    def getJointMovementStatus(self):
        """Returns the finished moving and under load threshold statuses of a this joint"""
        allMoving = True
        allUnderLoadThreshold = True
        for _, motor in enumerate(self._motors):
            motorMoving, motorUnderLoadThreshold = motor.getMotorMovementStatus()
            allMoving = allMoving and motorMoving
            allUnderLoadThreshold = allUnderLoadThreshold and motorUnderLoadThreshold
        return allMoving, allUnderLoadThreshold
    
    def getTransform(self):
        """Return the symbolic transform from the base of this joint to the next"""
        return self._transform
    
    def toggleActivate(self, activate):
        """Toggles the activation (torque) state of this joint"""
        for _, motor in enumerate(self._motors):
            motor.toggleTorque(activate)

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
            for _, motor in enumerate(self._motors):
                newPos = motor.posFromRad(theta)
                motor.safeSendMotorPosition(newPos)
                print('Setting {} to angle {}'.format(self._qID, theta))
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
    def __init__(self, gripperLimits, tuningParameters, portHandler, packetHandler, groupBulkWrite, groupSyncReadPos, groupSyncReadLoad, errorCallback):
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
            self._motor.sendMotorPosition(gripMotorPos)
        else:
            print('Cannot send gripper to out of bounds theta: {}'.format(gripMotorPos))

    #============================= Gripper Access ==================================
    def readGripperPosition(self):
        gripMotorPosition = self._motor.readMotorPos()
        return (gripMotorPosition - self._gripper_lims[0]) / self.getGripperRange()
    