from dataclasses import dataclass
from typing import List
import copy
import yaml

@dataclass
class MotorConfig:
    #Dynamixel defined ID for this motor
    id: int
    #Set of two (motorPosition, angle) pairs to define the angle/position mapping for this motor (compensates for badly aligned motors and gear ratios)
    samples: List[List[float]]
    #p parameter of PID control - converted by division by 128
    p: int = 640
    #i parameter of PID control - converted by division by 65536
    i: int = 0
    #i parameter of PID control - converted by division by 16
    d: int = 3600
    #acceleration gain of feed forward control - converted by division by 4
    ff2: int = 0
    #velocity gain of feed forward control - converted by division by 4
    ff1: int = 0

def getArmConfiguration():
    """Get arm configuration from the local config file and unpack it into a dataclass"""
    with open('config.yml', 'r') as file:
        rawConfig = yaml.safe_load(file)
        newConfig = copy.deepcopy(rawConfig)
        #Marshal rawConfig into MotorConfig dataclasses

        #arm motor configs
        for i, jointConfig in enumerate(rawConfig['arm']['joints']):
            for j, rawMotorConfig in enumerate(jointConfig['motors']):
                newConfig['arm']['joints'][i]['motors'][j] = MotorConfig(rawMotorConfig['id'], rawMotorConfig['samples'], rawMotorConfig['p'], rawMotorConfig['i'], rawMotorConfig['d'], rawMotorConfig['ff2'], rawMotorConfig['ff1'])

        #gripper motor config
        rawMotorConfig = rawConfig['arm']['gripper']['motor']
        newConfig['arm']['gripper']['motor'] = MotorConfig(rawMotorConfig['id'], rawMotorConfig['samples'], rawMotorConfig['p'], rawMotorConfig['i'], rawMotorConfig['d'], rawMotorConfig['ff2'], rawMotorConfig['ff1'])

        return newConfig
    
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
