import wpilib
from magicbot import StateMachine, state, timed_state
from magicbot import tunable
import ctre

import components.arm_controller
import components.drivetrain
import components.hatch_ready
import components.hatch_grab
# import robot


class HatchGrabber:
    '''
        Hatch Grabber
    '''
    arm_controller: components.arm_controller.ArmController
    drivetrain: components.drivetrain.Drivetrain

    hatch_grabber: ctre.WPI_VictorSPX
    hatch_grabber_switch: wpilib.DigitalInput

    # get_ready: components.hatch_ready.GetReady
    # grab_hatch: components.hatch_grab.GrabHatch

    motor_positive = tunable(default=0)
    motor_negative = tunable(default=0)

    def __init__(self):
        # self.motor_positive = 0
        # self.motor_negative = 0

        self.drive_y = 0

        self.get_grabber_ready = False
        self.grab_the_hatch = False
        self.move_arm = False

        # self.get_ready: GetReady
        # self.grab_hatch: GrabHatch

    def setup(self):
        # self.get_ready: GetReady
        # self.grab_hatch: GrabHatch
        self.get_ready: components.hatch_ready.GetReady
        self.grab_hatch: components.hatch_grab.GrabHatch
    
    def ready_the_grabber(self):
        self.get_grabber_ready = True
        self.grab_the_hatch = False
    
    def grab(self):
        self.get_grabber_ready = False
        self.grab_the_hatch = True
    
    def stow(self):
        self.motor_positive = 0
        self.motor_negative = 0.25

        self.get_grabber_ready = False
        self.grab_the_hatch = False
    
    def set_motor(self, positive, negative):
        self.motor_positive = positive
        self.motor_negative = negative

    def execute(self):
        if self.get_grabber_ready is True:
            self.get_ready.run()
        
        if self.grab_the_hatch is True:
            self.grab_hatch.run()

        if self.hatch_grabber_switch.get() is True:
            self.hatch_grabber.set(self.motor_positive - self.motor_negative)
        else:
            self.hatch_grabber.set(self.motor_positive)
        
        if self.drive_y is not 0:
            self.drivetrain.move(self.drive_y, 0, 0)
        
        if self.move_arm is True:
            self.arm_controller.set_arm_position(12, 0)
            self.move_arm = False
