import wpilib
from magicbot import tunable

import numpy as np

import components.arm


LENGTH_BETWEEN_JOINTS = 28


class ArmController:
    '''
        Robot Arm Controller
    '''

    arm: components.arm.Arm

    cone_x = tunable(default=12)
    cone_y = tunable(default=0)

    low_joint_angle = tunable(default=0)
    middle_joint_angle = tunable(default=0)
    high_joint_angle = tunable(default=0)
    
    def __init__(self):
        pass

    def setup(self):
        pass
    
    def set_arm_position(self, x, y):
        self.cone_x = x
        self.cone_y = y

    def execute(self):
        if (self.cone_x > 0):
            self.middle_joint_angle = -np.degrees(np.arccos((self.cone_x**2 + self.cone_y**2 - 2 * LENGTH_BETWEEN_JOINTS**2)/(2 * LENGTH_BETWEEN_JOINTS**2)))
            self.low_joint_angle = -(np.degrees(np.arctan(-self.cone_y / self.cone_x)) + np.degrees(np.arctan((LENGTH_BETWEEN_JOINTS * np.sin(self.middle_joint_angle * np.pi / 180)) / (LENGTH_BETWEEN_JOINTS + LENGTH_BETWEEN_JOINTS * np.cos(self.middle_joint_angle * np.pi / 180)))))
            self.high_joint_angle = self.low_joint_angle + self.middle_joint_angle + 80
        else:
            self.middle_joint_angle = +np.degrees(np.arccos((self.cone_x**2 + self.cone_y**2 - 2 * LENGTH_BETWEEN_JOINTS**2)/(2 * LENGTH_BETWEEN_JOINTS**2)))
            self.low_joint_angle = 180 - (np.degrees(np.arctan(-self.cone_y / self.cone_x)) + np.degrees(np.arctan((LENGTH_BETWEEN_JOINTS * np.sin(self.middle_joint_angle * np.pi / 180)) / (LENGTH_BETWEEN_JOINTS + LENGTH_BETWEEN_JOINTS * np.cos(self.middle_joint_angle * np.pi / 180)))))
            self.high_joint_angle = self.low_joint_angle + self.middle_joint_angle - 80
        self.arm.set_arm_setpoint(self.low_joint_angle, self.middle_joint_angle, self.high_joint_angle)
