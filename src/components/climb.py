import wpilib
from magicbot import StateMachine, state, timed_state
from magicbot import tunable
import ctre

import numpy as np

# import robot
# import components.drivetrain


class Climb(StateMachine):
    '''
        Climb mechanism
    '''

    drive: wpilib.drive.MecanumDrive

    front_left_climb: ctre.WPI_VictorSPX
    front_right_climb: ctre.WPI_VictorSPX
    rear_climb: ctre.WPI_VictorSPX
    rear_climb_drive: ctre.WPI_VictorSPX

    aborted = False

    # low_p = tunable(default=0.0015)


    # def __init__(self):
    #     # pass
    #     self.aborted = False

    # def setup(self):
    #     pass
    #     # self.drivetrain: components.drivetrain.Drivetrain
    
    def run(self):
        self.engage()
    
    # def stop_lifting(self):
    #     self.next_state_now('drive_forward_one')
    
    def abort(self):
        self.aborted = True

        # self.next_state_now('nothing')
    
    def stop_motors(self):
        self.front_left_climb.set(0)
        self.front_right_climb.set(0)
        self.rear_climb.set(0)
        self.rear_climb_drive.set(0)

        self.drive.driveCartesian(0, 0, 0)
    
    @timed_state(first=True, duration=3.5, next_state='drive_forward_one')
    # @timed_state(first=True, duration=2, next_state='nothing')
    def lift(self):
        self.front_left_climb.set(0.35)
        self.front_right_climb.set(0.45)
        self.rear_climb.set(0.37)
        self.rear_climb_drive.set(0)

        self.drive.driveCartesian(0, 0, 0)

        if self.aborted:
            self.stop_motors()
            self.next_state('nothing')
        # else:
        #     self.next_state('drive_forward_one')

    @timed_state(duration=2, next_state='raise_front')
    def drive_forward_one(self):
        self.front_left_climb.set(0)
        self.front_right_climb.set(0)
        self.rear_climb.set(0)
        self.rear_climb_drive.set(1)

        self.drive.driveCartesian(-0.1, 0, 0)

        if self.aborted:
            self.stop_motors()
            self.next_state('nothing')
        # else:
        #     self.next_state('raise_front')
    
    @timed_state(duration=2, next_state='drive_forward_two')
    def raise_front(self):
        self.front_left_climb.set(-0.15)
        self.front_right_climb.set(-0.15)
        self.rear_climb.set(0)
        self.rear_climb_drive.set(0)

        self.drive.driveCartesian(0, 0, 0)

        if self.aborted:
            self.stop_motors()
            self.next_state('nothing')
        # else:
        #     self.next_state('drive_forward_two')
    
    @timed_state(duration=2, next_state='raise_back')
    def drive_forward_two(self):
        self.front_left_climb.set(0)
        self.front_right_climb.set(0)
        self.rear_climb.set(0)
        self.rear_climb_drive.set(1)

        self.drive.driveCartesian(-0.1, 0, 0)

        if self.aborted:
            self.stop_motors()
            self.next_state('nothing')
        # else:
        #     self.next_state('raise_back')
    
    @timed_state(duration=2, next_state='drive_forward_three')
    def raise_back(self):
        self.front_left_climb.set(0)
        self.front_right_climb.set(0)
        self.rear_climb.set(-0.15)
        self.rear_climb_drive.set(0)

        self.drive.driveCartesian(0, 0, 0)

        if self.aborted:
            self.stop_motors()
            self.next_state('nothing')
        # else:
        #     self.next_state('drive_forward_three')
    
    @timed_state(duration=2, next_state='nothing')
    def drive_forward_three(self):
        self.front_left_climb.set(0)
        self.front_right_climb.set(0)
        self.rear_climb.set(0)
        self.rear_climb_drive.set(0)

        self.drive.driveCartesian(-0.1, 0, 0)
        
        # self.next_state('nothing')
    
    @state()
    def nothing(self):
        self.front_left_climb.set(0)
        self.front_right_climb.set(0)
        self.rear_climb.set(0)
        self.rear_climb_drive.set(0)

        self.drive.driveCartesian(0, 0, 0)
