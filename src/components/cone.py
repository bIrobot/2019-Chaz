import wpilib
import ctre


class Cone:
    '''
        Cone Controller
    '''

    cone_motor: ctre.WPI_VictorSPX
    cone_motor_encoder: wpilib.Encoder
    cone_switch: wpilib.DigitalInput

    def __init__(self):
        self.extended = False
        self.retracted = False
        self.release_hatch_panel = False
        self.first_release = True

        self.extend_limit = 50000
        self.retract_limit = 0

    def setup(self):
        self.cone_motor_encoder.reset()

    def release(self):
        self.release_hatch_panel = True

    def execute(self):
        if self.cone_switch.get() is False and self.release_hatch_panel is True and self.first_release is True:
            self.cone_motor_encoder.reset()
            self.first_release = False

        if self.cone_motor_encoder.get() >= self.extend_limit:
            self.extended = True
            self.retracted = False
        elif self.cone_motor_encoder.get() <= self.retract_limit and self.first_release is False:
            self.extended = False
            self.retracted = True
        else:
            self.extended = False
            self.retracted = False
        
        if self.extended is True and self.release_hatch_panel is False:
            self.cone_motor.set(0)

        if self.release_hatch_panel is True:
            self.cone_motor.set(0.35)
        
        if self.retracted is True and self.first_release is False:
            self.release_hatch_panel = False
            self.cone_motor.set(-0.2)