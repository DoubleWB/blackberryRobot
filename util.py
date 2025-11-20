from dynamixel_sdk import COMM_SUCCESS
import copy
import yaml
import numpy as np
import math
from sympy import Matrix

POLLING_DELAY = 0.050
MOVEMENT_TIME_LIMIT = 2.5
CONFIG_FILE_NAME = 'config.yml'
POSE_FILE_NAME = 'poses.yml'

class MotorConfig:
    def __init__(self, id, zeroPos, radsPerPos = 0.00153398193, p = 640, i = 0, d = 3600, ff2 = 0, ff1 = 0):
        #Dynamixel defined ID for this motor
        self.id = id
        #Position at which the angle of this motor is 0
        self.zeroPos = zeroPos
        #The number of radians per pos increment (can be negative to represent reversed motors)
        self.radsPerPos = radsPerPos
        #p parameter of PID control - converted by division by 128
        self.p = p
        #i parameter of PID control - converted by division by 65536
        self.i = i
        #d parameter of PID control - converted by division by 16
        self.d = d
        #acceleration gain of feed forward control - converted by division by 4
        self.ff2 = ff2
        #velocity gain of feed forward control - converted by division by 4
        self.ff1 = ff1

def getArmConfiguration():
    """Get arm configuration from the local config file and unpack it into a dataclass"""
    with open(CONFIG_FILE_NAME, 'r') as file:
        rawConfig = yaml.safe_load(file)
        newConfig = copy.deepcopy(rawConfig)
        #Marshal rawConfig into MotorConfig dataclasses

        #arm motor configs
        for i, jointConfig in enumerate(rawConfig['arm']['joints']):
            for j, rawMotorConfig in enumerate(jointConfig['motors']):
                newConfig['arm']['joints'][i]['motors'][j] = MotorConfig(rawMotorConfig['id'], rawMotorConfig['zeroPos'], rawMotorConfig['radsPerPos'], rawMotorConfig['p'], rawMotorConfig['i'], rawMotorConfig['d'], rawMotorConfig['ff2'], rawMotorConfig['ff1'])

        #gripper motor config
        rawMotorConfig = rawConfig['arm']['gripper']['motor']
        newConfig['arm']['gripper']['motor'] = MotorConfig(rawMotorConfig['id'], rawMotorConfig['zeroPos'], rawMotorConfig['radsPerPos'], rawMotorConfig['p'], rawMotorConfig['i'], rawMotorConfig['d'], rawMotorConfig['ff2'], rawMotorConfig['ff1'])

        return newConfig
    
def showPoseNamesFromFile():
    """Reads and displays the names of the poses in the local poses file"""
    with open(POSE_FILE_NAME, 'r') as file:
        print('Saved poses: ')
        posesRaw = yaml.safe_load(file)
        for pose in posesRaw['poses']:
            print(pose['poseName'])

def readPoseFromFile(poseName):
    """Read the saved set of q angles associated with the given pose name from the local poses file, and None if it doesn't exist"""
    with open(POSE_FILE_NAME, 'r') as file:
        posesRaw = yaml.safe_load(file)
        for pose in posesRaw['poses']:
            if pose['poseName'] == poseName:
                return pose['q']
        return None
    
def savePoseToFile(q, poseName):
    """Write the given set of q angles to the local poses file under the given pose name"""
    posesRaw = {}
    with open(POSE_FILE_NAME, 'r') as file:
        posesRaw = yaml.safe_load(file)
    if (len(q) == 6):
        allPoses = posesRaw['poses']
        newPose = {'poseName': poseName, 'q': q}
        allPoses.append(newPose)
        with open(POSE_FILE_NAME, 'w') as file:
            yaml.dump(posesRaw, file, default_flow_style=False, sort_keys=False)

def handleCommResponse(comm_result, command_string, packetHandler, verbose = False):
    """Handle the repeated logic of checking the result of a dynamixel communication, and printing success/failure with the given command string. Also returns True when the communication was successful."""
    if comm_result != COMM_SUCCESS:
        print('Dynamixel command failure: {} | {}'.format(command_string, packetHandler.getTxRxResult(comm_result)))
    elif verbose:
        print('Dynamixel command success: {}'.format(command_string))
    return comm_result == COMM_SUCCESS

def sendAndClearBulkWrite(command_string, groupBulkWrite, packetHandler):
    """Send a groupBulkWrite pending packet of params, and handle logging the result with the given command string before clearing the groupBulkWrite"""
    dxl_comm_result = groupBulkWrite.txPacket()
    handleCommResponse(dxl_comm_result, command_string, packetHandler)
    groupBulkWrite.clearParam()
    
def convertPosToSigned(n):
    """Converts 2 byte unsigned integers returned from load reading to their signed values"""
    # n: unsigned 32-bit integer (0..4294967295)
    if n >= 0x80000000:  # If the sign bit is set
        return n - 0x100000000
    else:
        return n
    
def convertLoadToSigned(n):
    """Converts 2 byte unsigned integers returned from load reading to their signed values"""
    # n: unsigned 16-bit integer (0..65535)
    if n >= 0x8000:  # If the sign bit is set
        return n - 0x10000
    else:
        return n
    
def getGearToothAlignmentFromPos(pos):
    """Converts dynamixel motor pos value [0, 4095] to 12 tooth gear alignment, [0 (the lowest point in the gear), 1 (the apex of a gear tooth)]"""
    #convert pos to radians
    posInRad = pos * ((2 * np.pi)/4096)
    #The gear in question has 12 teeth, the period of a tooth apex occuring is 2pi/12 = pi/6
    #The gear starts at a low point at 0 radians, so this can be modeled by a cosine of the appropriate period
    return (-math.cos(posInRad * 12) + 1)/2

def getClosestGearToothAlignmentPos(alignmentVal, initPos):
    """Returns the nearest dynamixel motor pos value to initPos with the given alignmentVal"""
    numPeriods = 24 # Number of full rotations (2) times the number of teeth (12) to check for solutions in one direction
    #One direction will cover pi/2 radians - half of the pi radians range of the geared joint after a 12:43 gear ratio is applied
    firstSolution = math.acos(1 - (alignmentVal*2))
    initPosInRad = initPos * ((2 * np.pi)/4096)
    bestSolution = None
    for n in range(-numPeriods, numPeriods):
        # 2 solutions per period
        sol1 = (firstSolution + (2 * np.pi * n))/12
        sol2 = (-firstSolution + (2 * np.pi * n))/12    
        if (bestSolution == None) or (abs(initPosInRad - sol1) < abs(initPosInRad - bestSolution)):
            bestSolution = sol1
        if abs(initPosInRad - sol2) < abs(initPosInRad - bestSolution):
            bestSolution = sol2
    return round(bestSolution * 4096/(2 * np.pi))

def getErrorVector(pose1, pose2):
    """Return the column vector of errors between two """
    if len(pose1) == 6 and len(pose2) == 6:
        return Matrix([[pose1[0]-pose2[0]], [pose1[1]-pose2[1]], [pose1[2]-pose2[2]], [pose1[3]-pose2[3]], [pose1[4]-pose2[4]], [pose1[5]-pose2[5]]])
    print('Bad input - poses not 6DOF.')
    return Matrix([[]])

def getMagnitude(vec):
    """Return the magnitude of the given vector of values"""
    squareSums = 0
    for error in vec:
        squareSums += error ** 2
    return math.sqrt(squareSums)
    
