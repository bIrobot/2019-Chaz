import wpilib
from magicbot import tunable
import ctre

import numpy as np


class Arm:
    '''
        Robot Arm
    '''

    actuator: ctre.WPI_VictorSPX
    actuator_switch_min: wpilib.DigitalInput
    actuator_switch_max: wpilib.DigitalInput

    # low_p = tunable(default=0.0015)


    def __init__(self):
        self.set_drive_position = False
        self.set_start_position = False
        self.in_start_position = True
        self.set_actuator = True

    def setup(self):
        pass
    
    def drive_position(self):
        self.set_drive_position = True
    
    def start_position(self):
        self.set_start_position = True
    
    def enable(self, value):
        self.set_actuator = value

    def execute(self):
        if self.actuator_switch_min.get() is False and self.set_start_position is True:
            self.set_start_position = False
        
        if self.actuator_switch_max.get() is False and self.set_drive_position is True:
            self.set_drive_position = False
            self.in_start_position = False
        
        if self.set_actuator is True:
            if self.set_start_position is True and self.in_start_position is False:
                self.actuator.set(-0.3)
            elif self.set_drive_position is True:
                self.actuator.set(0.25)
            else:
                self.actuator.set(0)
