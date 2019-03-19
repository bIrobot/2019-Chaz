import wpilib
from magicbot import StateMachine, state, timed_state
from magicbot import tunable
import ctre

import components.hatch_grabber


class GetReady(StateMachine):
    

    def setup(self):
        self.hatchgrabber: components.hatch_grabber.HatchGrabber

    def run(self):
        self.engage()

        self.hatchgrabber.set_motor(0.25, 0)
    
    @state(first=True)
    def set_arm(self):
        self.hatchgrabber.move_arm = True

        self.next_state_now('lower')
    
    @timed_state(duration=2)
    def lower(self):
        self.hatchgrabber.set_motor(0.25, 0)
        # self.hatchgrabber.motor_positive = 0.25
        # self.hatchgrabber.motor_negative = 0

        self.next_state('hold')
    
    @state()
    def hold(self):
        # HatchGrabber.set_motor(0, 0.1)
        self.hatchgrabber.motor_positive = 0
        self.hatchgrabber.motor_negative = 0.1