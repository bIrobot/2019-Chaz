import wpilib
import ctre
from magicbot import StateMachine, state, timed_state
import components.drivetrain

class HatchGrabber:
    '''
        Hatch Grabber
    '''

    hatch_grabber: ctre.WPI_VictorSPX
    hatch_grabber_switch: wpilib.DigitalInput

    def __init__(self):
        self.motor_positive = 0
        self.motor_negative = 0

        self.get_grabber_ready = False
        self.grab_the_hatch = False

    def setup(self):
        self.get_ready: GetReady
        self.grab_hatch: GrabHatch
    
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

    def execute(self):
        if self.get_grabber_ready is True:
            self.get_ready.run()
        
        if self.grab_the_hatch is True:
            self.grab_hatch.run()

        if self.hatch_grabber_switch.get() is True:
            self.hatch_grabber.set(self.motor_positive - self.motor_negative)
        else:
            self.hatch_grabber.set(self.motor_positive)


class GetReady(StateMachine):

    def run(self):
        self.engage()
    
    @timed_state(first=True, duration=2)
    def lower(self):
        HatchGrabber.motor_positive = 0.25
        HatchGrabber.motor_negative = 0

        self.next_state('hold')
    
    @state()
    def hold(self):
        HatchGrabber.motor_positive = 0
        HatchGrabber.motor_negative = 0.1


class GrabHatch(StateMachine):

    drivetrain: components.drivetrain.Drivetrain
    
    def run(self):
        self.engage()
    
    @timed_state(first=True, duration=1)
    def lift(self):
        HatchGrabber.motor_positive = 0
        HatchGrabber.motor_negative = 0.25

        self.next_state('reverse')
    
    @timed_state(duration=1)
    def reverse(self):
        self.drivetrain.move(-0.1, 0, 0)

        self.next_state('slam')
    
    @timed_state(duration=2)
    def slam(self):
        HatchGrabber.motor_positive = 0.75
        HatchGrabber.motor_negative = 0

        self.next_state('release')
    
    @timed_state(duration=2)
    def release(self):
        HatchGrabber.motor_positive = 0
        HatchGrabber.motor_negative = 1
