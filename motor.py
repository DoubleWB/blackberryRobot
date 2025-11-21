from dynamixel_sdk import * # Uses Dynamixel SDK library
import util

# Dynamixel constants for X_SERIES
ADDR_POSITION_P             = 84
ADDR_POSITION_I             = 82
ADDR_POSITION_D             = 80
LEN_POSITION_PID            = 2
ADDR_FF_ACC_GAIN            = 88    # Feedforward second gain
ADDR_FF_VEL_GAIN            = 90    # Feedforward first gain
LEN_FF_GAIN                 = 2
ADDR_DRIVE_MODE             = 10
LEN_DRIVE_MODE              = 1
ADDR_OP_MODE                = 11
LEN_OP_MODE                 = 1
ADDR_ACC_PROF               = 108
LEN_ACC_PROF                = 4
ADDR_VEL_PROF               = 112
LEN_VEL_PROF                = 4
ADDR_TORQUE_ENABLE          = 64
ADDR_LED_RED                = 65
LEN_LED_RED                 = 1
ADDR_GOAL_POSITION          = 116
LEN_GOAL_POSITION           = 4
ADDR_PRESENT_POSITION       = 132
LEN_PRESENT_POSITION        = 4
ADDR_PRESENT_LOAD           = 126
LEN_PRESENT_LOAD            = 2
TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque

class DynamixelMotor:
    #============================= Motor Constants ==================================
    
    _vel_time_profile = 1200 #ms
    _acc_time_profile = 750 #ms
    _pos_epsilon = 35 #~3 Degrees
    _load_threshold = 500 #50%


    #============================= DynamixelMotor Construction ==================================
    def __init__(self, tuningParameters, portHandler, packetHandler, groupBulkWrite, groupSyncReadPos, groupSyncReadLoad, errorCallback):
        """Initialize a connection to the robot with inverse kinematic control"""

        # Initialize DynamixelCommunication
        self._portHandler = portHandler
        self._packetHandler = packetHandler
        self._groupBulkWrite = groupBulkWrite
        self._groupSyncReadPos = groupSyncReadPos
        self._groupSyncReadLoad = groupSyncReadLoad

        # Intialize Motor Parameters
        self._dID = tuningParameters.id

        #Setup/Tune Motor
        self._tuningParameters = tuningParameters
        self.setZeroPos(tuningParameters.zeroPos)

        #Store error callback function
        self._errorCallback = errorCallback

        #Initialize memory for last commanded position
        self.configureMotorReads()  # Setup read parameters to get motor position during initialization
        self._lastCommandedPos = self.readMotorPos()

        self.setPositionPID()
        self.setPositionFF()
        self.setModes()
        self.setTimeProfile()

    #============================= Comm Utilities ==========================================
    def configureMotorReads(self):
        """Initialize the groupBulkRead with all the parameters we want to read from the given dID"""
        dxl_addparam_result = self._groupSyncReadPos.addParam(self._dID)
        if dxl_addparam_result != True:
            print('Read pos param setting failed for dynamixel {}'.format(self._dID))
        else:
            print('Read pos param set for dynamixel {}'.format(self._dID))
        dxl_addparam_result = self._groupSyncReadLoad.addParam(self._dID)
        if dxl_addparam_result != True:
            print('Read load param setting failed for dynamixel {}'.format(self._dID))
        else:
            print('Read load param set for dynamixel {}'.format(self._dID))

    #=========================== Motor Utilities ===========================================
    def toggleTorque(self, enable):
        """Toggle the torque for a single motor"""
        toggleParam = TORQUE_ENABLE if enable else TORQUE_DISABLE
        dxl_comm_result, dxl_error = self._packetHandler.write1ByteTxRx(self._portHandler, self._dID, ADDR_TORQUE_ENABLE, toggleParam)
        util.handleCommResponse(dxl_comm_result, 'Motor {} torque toggled {}'.format(self._dID, toggleParam), self._packetHandler)
        #Additional error handling potential just for single byte writes
        if dxl_error != 0:
            print('Torque toggle error: {}'.format(self._packetHandler.getRxPacketError(dxl_error)))

    def setPositionPID(self):
        """Sets the PID position parameters for a single motor"""
        #P
        param_position_p = [DXL_LOBYTE(self._tuningParameters.p), DXL_HIBYTE(self._tuningParameters.p)]
        self._groupBulkWrite.addParam(self._dID, ADDR_POSITION_P, LEN_POSITION_PID, param_position_p)
        util.sendAndClearBulkWrite('Motor {} P set to: {}'.format(self._dID, self._tuningParameters.p), self._groupBulkWrite, self._packetHandler)
        #I
        param_position_i = [DXL_LOBYTE(self._tuningParameters.i), DXL_HIBYTE(self._tuningParameters.i)]
        self._groupBulkWrite.addParam(self._dID, ADDR_POSITION_I, LEN_POSITION_PID, param_position_i)
        util.sendAndClearBulkWrite('Motor {} I set to: {}'.format(self._dID, self._tuningParameters.i), self._groupBulkWrite, self._packetHandler)
        #D
        param_position_d = [DXL_LOBYTE(self._tuningParameters.d), DXL_HIBYTE(self._tuningParameters.d)]
        self._groupBulkWrite.addParam(self._dID, ADDR_POSITION_D, LEN_POSITION_PID, param_position_d)
        util.sendAndClearBulkWrite('Motor {} D set to: {}'.format(self._dID, self._tuningParameters.d), self._groupBulkWrite, self._packetHandler)

    def setPositionFF(self):
        """Sets the feedforward gain parameters for a single motor"""
        #FF2
        param_ff_acc = [DXL_LOBYTE(self._tuningParameters.ff2), DXL_HIBYTE(self._tuningParameters.ff2)]
        self._groupBulkWrite.addParam(self._dID, ADDR_FF_ACC_GAIN, LEN_FF_GAIN, param_ff_acc)
        util.sendAndClearBulkWrite('Motor {} FF2 (acc) set to: {}'.format(self._dID, self._tuningParameters.ff2), self._groupBulkWrite, self._packetHandler)
        #FF1
        param_ff_vel = [DXL_LOBYTE(self._tuningParameters.ff1), DXL_HIBYTE(self._tuningParameters.ff1)]
        self._groupBulkWrite.addParam(self._dID, ADDR_FF_VEL_GAIN, LEN_FF_GAIN, param_ff_vel)
        util.sendAndClearBulkWrite('Motor {} FF1 (vel) set to: {}'.format(self._dID, self._tuningParameters.ff1), self._groupBulkWrite, self._packetHandler)

    def setModes(self):
        """Sets the drive and operating modes for a single motor"""
        #Drive Mode 4 = time profile
        self._groupBulkWrite.addParam(self._dID, ADDR_DRIVE_MODE, LEN_DRIVE_MODE, [4])
        util.sendAndClearBulkWrite('Motor {} drive mode set to time profile'.format(self._dID), self._groupBulkWrite, self._packetHandler)
        #Operating Mode 4 = extended position control - allows sending of negative positions to rotate clockwise
        self._groupBulkWrite.addParam(self._dID, ADDR_OP_MODE, LEN_OP_MODE, [4])
        util.sendAndClearBulkWrite('Motor {} operating mode set to extended position control'.format(self._dID), self._groupBulkWrite, self._packetHandler)

    def setTimeProfile(self):
        """Sets the motor profile velocity and acceleration time limits for a single motor"""
        #Velocity Time Limit
        param_vel_time = [DXL_LOBYTE(DXL_LOWORD(self._vel_time_profile)), DXL_HIBYTE(DXL_LOWORD(self._vel_time_profile)), DXL_LOBYTE(DXL_HIWORD(self._vel_time_profile)), DXL_HIBYTE(DXL_HIWORD(self._vel_time_profile))]
        self._groupBulkWrite.addParam(self._dID, ADDR_VEL_PROF, LEN_VEL_PROF, param_vel_time)
        util.sendAndClearBulkWrite('Motor {} velocity time profile set to: {}'.format(self._dID, self._vel_time_profile), self._groupBulkWrite, self._packetHandler)
        #Acceleration Time Limit
        param_acc_time = [DXL_LOBYTE(DXL_LOWORD(self._acc_time_profile)), DXL_HIBYTE(DXL_LOWORD(self._acc_time_profile)), DXL_LOBYTE(DXL_HIWORD(self._acc_time_profile)), DXL_HIBYTE(DXL_HIWORD(self._acc_time_profile))]
        self._groupBulkWrite.addParam(self._dID, ADDR_ACC_PROF, LEN_ACC_PROF, param_acc_time)
        util.sendAndClearBulkWrite('Motor {} acceleration time profile set to: {}'.format(self._dID, self._acc_time_profile), self._groupBulkWrite, self._packetHandler)

    def setZeroPos(self, zeroPos):
        """Given a position at which this joint is 0 degrees, and set the radians at which pos 0 lands (the radian intercept)"""
        self._radIntercept = -zeroPos * self._tuningParameters.radsPerPos

    def getRadsPerPos(self):
        """Return the configured radians covered by one dynamixel position for this motor"""
        return self._tuningParameters.radsPerPos

    def radFromPos(self, pos):
        """Return the angle value associated with the payload of this motor when at the given position"""
        return (self._tuningParameters.radsPerPos * pos) + self._radIntercept

    def posFromRad(self, rad):
        """Return the position value associated with the payload of this motor when at the given angle"""
        return int((rad - self._radIntercept)/self._tuningParameters.radsPerPos)
    
    def getMotorMovementStatus(self):
        """Returns the finished moving and under load threshold statuses of a single motor"""
        #Worst case scenario by default, in case comms have failed
        moving = True
        underLoadThreshold = False
        dxl_pos_result = self._groupSyncReadPos.isAvailable(self._dID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        if dxl_pos_result == True:
            pos = util.convertPosToSigned(self._groupSyncReadPos.getData(self._dID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION))
            moving = abs(self._lastCommandedPos - pos) > self._pos_epsilon
        dxl_load_result = self._groupSyncReadLoad.isAvailable(self._dID, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD)
        if dxl_load_result == True:
            load = util.convertLoadToSigned(self._groupSyncReadLoad.getData(self._dID, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD))
            #useful for testing
            #print('{} current load is: {}'.format(self._dID, load))
            underLoadThreshold = abs(load) < self._load_threshold
        return moving, underLoadThreshold
    
    def fetchAndGetMotorMovementStatus(self):
        """Returns the finished moving and under load threshold statuses of a single motor after a fresh fetch"""
        dxl_comm_result = self._groupSyncReadPos.txRxPacket()
        readSuccess = util.handleCommResponse(dxl_comm_result, 'Motor {} status (position) read'.format(self._dID), self._packetHandler)
        dxl_comm_result = self._groupSyncReadLoad.txRxPacket()
        readSuccess = readSuccess and util.handleCommResponse(dxl_comm_result, 'Motor {} status (load) read'.format(self._dID), self._packetHandler)
        if readSuccess:
            return self.getMotorMovementStatus()
        #return worst case by default if comms have failed
        return True, False

    #=========================== Motor Control ===========================================
    def addMotorPosition(self, motorPos):
        """Adds a new position to the bulk writer for a single motor. Does not clear previous queued params"""
        self._lastCommandedPos = motorPos
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(motorPos)), DXL_HIBYTE(DXL_LOWORD(motorPos)), DXL_LOBYTE(DXL_HIWORD(motorPos)), DXL_HIBYTE(DXL_HIWORD(motorPos))]
        self._groupBulkWrite.addParam(self._dID, ADDR_GOAL_POSITION, LEN_GOAL_POSITION, param_goal_position)
    
    def sendMotorPosition(self, motorPos):
        """Sends a new position for a single motor"""
        self.addMotorPosition(motorPos)
        util.sendAndClearBulkWrite('Motor {} position set to: {}'.format(self._dID, motorPos), self._groupBulkWrite, self._packetHandler)

    def safeSendMotorPosition(self, motorPos):
        """Sends a new position for a single motor, and monitors for moving over the load threshold while achieving the position"""
        self.sendMotorPosition(motorPos)
        moving, underLoadThresh = self.fetchAndGetMotorMovementStatus()
        polls = 0
        #If enough polls elapse to cover the _movement_time_limit and no other issues have occured, we might have trouble reaching this pose for config reasons
        while moving and polls < util.MOVEMENT_TIME_LIMIT/util.POLLING_DELAY:
            if not underLoadThresh:
                self._errorCallback.__call__()
            time.sleep(util.POLLING_DELAY)
            moving, underLoadThresh = self.fetchAndGetMotorMovementStatus()
            polls += 1 

    def readMotorPos(self):
        """Returns the position value read for a single motor. Can return None on comms fail."""
        dxl_comm_result = self._groupSyncReadPos.txRxPacket()
        readSuccess = util.handleCommResponse(dxl_comm_result, 'Motor {} position read'.format(self._dID), self._packetHandler)
        if readSuccess:
            dxl_getdata_result = self._groupSyncReadPos.isAvailable(self._dID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if dxl_getdata_result == True:
                return util.convertPosToSigned(self._groupSyncReadPos.getData(self._dID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION))
                
        print('Motor {} position fetch failed'.format(self._dID))
        return None
    
    def readMotorLoad(self):
        """Returns the load value read for a single motor. Can return None on comms fail."""
        dxl_comm_result = self._groupSyncReadLoad.txRxPacket()
        readSuccess = util.handleCommResponse(dxl_comm_result, 'Motor {} load read'.format(self._dID), self._packetHandler)
        if readSuccess:
            dxl_getdata_result = self._groupSyncReadLoad.isAvailable(self._dID, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD)
            if dxl_getdata_result == True:
                return util.convertLoadToSigned(self._groupSyncReadLoad.getData(self._dID, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD))
                
        print('Motor {} load fetch failed'.format(self._dID))
        return None
    
