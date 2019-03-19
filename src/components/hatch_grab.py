import wpilib
from magicbot import StateMachine, state, timed_state
from magicbot import tunable
import ctre

import components.hatch_grabber


class GrabHatch(StateMachine):
    # hatchgrabber: components.hatch_grabber.HatchGrabber

    def setup(self):
        self.hatchgrabber: components.hatch_grabber.HatchGrabber

    def run(self):
        self.engage()
    
    @timed_state(first=True, duration=1)
    def lift(self):
        self.hatchgrabber.motor_positive = 0
        self.hatchgrabber.motor_negative = 0.25

        self.next_state('reverse')
    
    @timed_state(duration=1)
    def reverse(self):
        self.hatchgrabber.drive_y = -0.1

        self.next_state('slam')
    
    @timed_state(duration=2)
    def slam(self):
        self.hatchgrabber.motor_positive = 0.75
        self.hatchgrabber.motor_negative = 0

        self.next_state('release')
    
    @timed_state(duration=2)
    def release(self):
        self.hatchgrabber.motor_positive = 0
        self.hatchgrabber.motor_negative = 1
