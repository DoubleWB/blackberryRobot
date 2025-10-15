from dynamixel_sdk import * # Uses Dynamixel SDK library

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
TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque

class DynamixelMotor:
    #============================= Motor Constants ==================================
    
    #Max position on a dynamixel before looping back to 0
    _vel_time_profile = 2000
    _acc_time_profile = 500

    #============================= DynamixelMotor Construction ==================================
    def __init__(self, tuningParameters, portHandler, packetHandler, groupBulkWrite, groupBulkRead):
        """Initialize a connection to the robot with inverse kinematic control"""

        # Initialize DynamixelCommunication
        self._portHandler = portHandler
        self._packetHandler = packetHandler
        self._groupBulkWrite = groupBulkWrite
        self._groupBulkRead = groupBulkRead

        # Intialize Motor Parameters
        self._dID = tuningParameters.id

        #Setup/Tune Motor
        self._tuningParemeters = tuningParameters

        posDiff = (tuningParameters.samples[0][1] - tuningParameters.samples[1][1])
        if posDiff != 0:
            self._radsPerPos = (tuningParameters.samples[0][0] - tuningParameters.samples[1][0]) / posDiff
        else:
            print('Motor {} unable to calculate angle to position conversion given samples, setting to 1'.format(self._dID))
            self._radsPerPos = 1

        self._radIntercept = tuningParameters.samples[0][0] - self._radsPerPos * tuningParameters.samples[0][1]

        self.setPositionPID()
        self.setPositionFF()
        self.setModes()
        self.setTimeProfile()

    #============================= Comm Utilities ==========================================
    def sendAndClearBulkWrite(self, command_string):
        """Send a groupBulkWrite pending packet of params, and handle logging the result with the given command string before clearing the groupBulkWrite"""
        dxl_comm_result = self._groupBulkWrite.txPacket()
        self.handleCommResponse(dxl_comm_result, command_string)
        self._groupBulkWrite.clearParam()

    def handleCommResponse(self, comm_result, command_string, verbose = True):
        """Handle the repeated logic of checking the result of a dynamixel communication, and printing success/failure with the given command string. Also returns True when the communication was successful."""
        if comm_result != COMM_SUCCESS:
            print('Dynamixel command failure: {} | {}'.format(command_string, self._packetHandler.getTxRxResult(comm_result)))
        elif verbose:
            print('Dynamixel command success: {}'.format(command_string))
        return comm_result == COMM_SUCCESS
    
    def configureMotorReads(self):
        """Initialize the groupBulkRead with all the parameters we want to read from the given dID"""
        dxl_addparam_result = self._groupBulkRead.addParam(self._dID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        if dxl_addparam_result != True:
            print('Read param setting failed for dynamixel {}'.format(self._dID))
        else:
            print('Read params set for dynamixel {}'.format(self._dID))

    def hasMotorFailed(self):
        """Return whether or not this motor has hit an error"""
        err = self._packetHandler.getLastRxPacketError()
        #Or is it not sufficient to get the last error - should we fetch once more with self._packetHandler.readRx() instead?
        return err != 0

    #=========================== Motor Utilities ===========================================
    def toggleTorque(self, enable):
        """Toggle the torque for a single motor"""
        toggleParam = TORQUE_ENABLE if enable else TORQUE_DISABLE
        dxl_comm_result, dxl_error = self._packetHandler.write1ByteTxRx(self._portHandler, self._dID, ADDR_TORQUE_ENABLE, toggleParam)
        self.handleCommResponse(dxl_comm_result, 'Motor {} torque toggled {}'.format(self._dID, toggleParam))
        #Additional error handling potential just for single byte writes
        if dxl_error != 0:
            print('Torque toggle error: {}'.format(self._packetHandler.getRxPacketError(dxl_error)))

    def setPositionPID(self):
        """Sets the PID position parameters for a single motor"""
        #P
        param_position_p = [DXL_LOBYTE(self._tuningParemeters.p), DXL_HIBYTE(self._tuningParemeters.p)]
        self._groupBulkWrite.addParam(self._dID, ADDR_POSITION_P, LEN_POSITION_PID, param_position_p)
        self.sendAndClearBulkWrite('Motor {} P set to: {}'.format(self._dID, self._tuningParemeters.p))
        #I
        param_position_i = [DXL_LOBYTE(self._tuningParemeters.i), DXL_HIBYTE(self._tuningParemeters.i)]
        self._groupBulkWrite.addParam(self._dID, ADDR_POSITION_I, LEN_POSITION_PID, param_position_i)
        self.sendAndClearBulkWrite('Motor {} I set to: {}'.format(self._dID, self._tuningParemeters.i))
        #D
        param_position_d = [DXL_LOBYTE(self._tuningParemeters.d), DXL_HIBYTE(self._tuningParemeters.d)]
        self._groupBulkWrite.addParam(self._dID, ADDR_POSITION_D, LEN_POSITION_PID, param_position_d)
        self.sendAndClearBulkWrite('Motor {} D set to: {}'.format(self._dID, self._tuningParemeters.d))

    def setPositionFF(self):
        """Sets the feedforward gain parameters for a single motor"""
        #FF2
        param_ff_acc = [DXL_LOBYTE(self._tuningParemeters.ff2), DXL_HIBYTE(self._tuningParemeters.ff2)]
        self._groupBulkWrite.addParam(self._dID, ADDR_FF_ACC_GAIN, LEN_FF_GAIN, param_ff_acc)
        self.sendAndClearBulkWrite('Motor {} FF2 (acc) set to: {}'.format(self._dID, self._tuningParemeters.ff2))
        #FF1
        param_ff_vel = [DXL_LOBYTE(self._tuningParemeters.ff1), DXL_HIBYTE(self._tuningParemeters.ff1)]
        self._groupBulkWrite.addParam(self._dID, ADDR_FF_VEL_GAIN, LEN_FF_GAIN, param_ff_vel)
        self.sendAndClearBulkWrite('Motor {} FF1 (vel) set to: {}'.format(self._dID, self._tuningParemeters.ff1))

    def setModes(self):
        """Sets the drive and operating modes for a single motor"""
        #Drive Mode 4 = time profile
        self._groupBulkWrite.addParam(self._dID, ADDR_DRIVE_MODE, LEN_DRIVE_MODE, [4])
        self.sendAndClearBulkWrite('Motor drive mode set to time profile')
        #Operating Mode 4 = extended position control - allows sending of negative positions to rotate clockwise
        self._groupBulkWrite.addParam(self._dID, ADDR_OP_MODE, LEN_OP_MODE, [4])
        self.sendAndClearBulkWrite('Motor operating mode set to extended position control')

    def setTimeProfile(self):
        """Sets the feedforward gain parameters for a single motor"""
        #Drive Mode 4 = time profile
        self._groupBulkWrite.addParam(self._dID, ADDR_DRIVE_MODE, LEN_DRIVE_MODE, [4])
        self.sendAndClearBulkWrite('Motor drive mode set to time profile')
        #Velocity Time Limit
        param_vel_time = [DXL_LOBYTE(DXL_LOWORD(self._vel_time_profile)), DXL_HIBYTE(DXL_LOWORD(self._vel_time_profile)), DXL_LOBYTE(DXL_HIWORD(self._vel_time_profile)), DXL_HIBYTE(DXL_HIWORD(self._vel_time_profile))]
        self._groupBulkWrite.addParam(self._dID, ADDR_VEL_PROF, LEN_VEL_PROF, param_vel_time)
        self.sendAndClearBulkWrite('Motor {} velocity time profile set to: {}'.format(self._dID, self._vel_time_profile))
        #Acceleration Time Limit
        param_acc_time = [DXL_LOBYTE(DXL_LOWORD(self._acc_time_profile)), DXL_HIBYTE(DXL_LOWORD(self._acc_time_profile)), DXL_LOBYTE(DXL_HIWORD(self._acc_time_profile)), DXL_HIBYTE(DXL_HIWORD(self._acc_time_profile))]
        self._groupBulkWrite.addParam(self._dID, ADDR_ACC_PROF, LEN_ACC_PROF, param_acc_time)
        self.sendAndClearBulkWrite('Motor {} acceleration time profile set to: {}'.format(self._dID, self._acc_time_profile))   

    def radFromPos(self, pos):
        """Return the angle value associated with the payload of this motor when at the given position"""
        return (self._radsPerPos * pos) + self._radIntercept

    def posFromRad(self, rad):
        """Return the position value associated with the payload of this motor when at the given angle"""
        return int((rad - self._radIntercept)/self._radsPerPos)

    #=========================== Motor Position Control ===========================================
    def addMotorPosition(self, motorPos):
        """Adds a new position to the bulk writer for a single motor. Does not clear previous queued params"""
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(motorPos)), DXL_HIBYTE(DXL_LOWORD(motorPos)), DXL_LOBYTE(DXL_HIWORD(motorPos)), DXL_HIBYTE(DXL_HIWORD(motorPos))]
        self._groupBulkWrite.addParam(self._dID, ADDR_GOAL_POSITION, LEN_GOAL_POSITION, param_goal_position)
    
    def sendMotorPosition(self, motorPos):
        """Sends a new position for a single motor"""
        self.addMotorPosition(motorPos)
        self.sendAndClearBulkWrite('Motor {} position set to: {}'.format(self._dID, motorPos))

    def readMotorPos(self):
        """Returns the position value read for a single motor. Can return None on comms fail."""
        dxl_comm_result = self.groupBulkRead.txRxPacket()
        readSuccess = self.handleCommResponse(dxl_comm_result, 'Motor {} position read'.format(self._dID))
        if readSuccess:
            dxl_getdata_result = self._groupBulkRead.isAvailable(self._dID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if dxl_getdata_result == True:
                return self._groupBulkRead.getData(self._dID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                
        print('Motor {} position fetch failed'.format(self._dID))
        return None