# !/usr/bin/env python3

import wpilib
import wpilib.drive
from wpilib.shuffleboard import Shuffleboard
from networktables import NetworkTables
import magicbot
import ctre
import navx

import fightstick
import components.arm
import components.climb
import components.cone
import components.drivetrain


class MyRobot(magicbot.MagicRobot):
    arm: components.arm.Arm
    climb: components.climb.Climb
    cone: components.cone.Cone
    drivetrain: components.drivetrain.Drivetrain

    use_teleop_in_autonomous = True

    def createObjects(self):
        """Initialize everything."""

        # Initialize joysticks
        self.controller = wpilib.XboxController(0)
        self.gamepad = fightstick.FightStick(1)

        wpilib.CameraServer.launch('vision.py:main')

        # NetworkTables.initialize()
        self.sd = NetworkTables.getTable("SmartDashboard")

        # Initialize drive
        self.front_left = ctre.WPI_TalonSRX(1)
        self.rear_left = ctre.WPI_TalonSRX(2)
        self.front_right = ctre.WPI_TalonSRX(3)
        self.rear_right = ctre.WPI_TalonSRX(4)
        
        self.drive = wpilib.drive.MecanumDrive(self.front_left, self.rear_left, self.front_right, self.rear_right)

        # Initialize motors other than drive
        self.actuator = ctre.WPI_VictorSPX(5)
        self.cone_motor = ctre.WPI_VictorSPX(6)
        self.front_left_climb = ctre.WPI_VictorSPX(7)
        self.front_right_climb = ctre.WPI_VictorSPX(8)
        self.rear_climb = ctre.WPI_VictorSPX(9)
        self.rear_climb_drive = ctre.WPI_VictorSPX(10)

        self.front_right_climb.setInverted(True)
        # self.arm_high.setInverted(True)
        
        # Initialize encoders
        # self.arm_low_encoder = wpilib.Encoder(0, 1)
        # self.arm_middle_encoder = wpilib.Encoder(2, 3)
        # self.arm_high_encoder = wpilib.Encoder(4, 5)
        self.cone_motor_encoder = wpilib.Encoder(0, 1)

        # Initialize switches and hall-effect sensors
        self.cone_switch = wpilib.DigitalInput(2)
        self.actuator_switch_min = wpilib.DigitalInput(3)
        self.actuator_switch_max = wpilib.DigitalInput(4)

        # Communicate w/navX MXP via the MXP SPI Bus.
        # - Alternatively, use the i2c bus.
        # See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details

        # self.navxSensor = navx.ahrs.AHRS.create_spi()
        # self.navxSensor = navx.ahrs.AHRS.create_i2c()

        self.climb_sequence_engaged = False
        self.climb_aborted = False

    def teleopInit(self):
        """This function is called at the beginning of teleoperated mode."""
        
        pass
    
    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""

        if self.climb_sequence_engaged is False:
            leftXAxis = self.normalize(self.controller.getX(0), 0.15)
            leftYAxis = self.normalize(self.controller.getY(0), 0.15)
            rotation = self.normalize(self.controller.getX(1), 0.15)

            self.drivetrain.move(leftYAxis, leftXAxis, rotation * 0.5)

            if self.controller.getTriggerAxis(1) > 0.5:
                self.cone.release()
            
            # value = (self.normalize(self.controller.getTriggerAxis(0), 0.15) + (self.normalize(self.controller.getTriggerAxis(1), 0.15) * -1)) * 0.5
            value = 0.0
            if value == 0.0:
                self.arm.enable(True)
                if self.controller.getAButton() is True:
                    self.arm.drive_position()
                elif self.controller.getBButton() is True:
                    self.arm.start_position()
            else:
                self.arm.enable(False)
                self.actuator.set(value)
            
            if self.controller.getBumper(0) is True and self.controller.getXButtonPressed() is True:
                self.arm.start_position()
                self.drivetrain.enable(False)
                self.climb.run()
                self.climb_sequence_engaged = True
        else:
            if self.controller.getAButtonPressed() is True:
                self.climb.abort()
                self.climb_aborted = True
            
            if self.climb_aborted is True:
                self.drivetrain.enable(True)
                self.climb_sequence_engaged = False
            
            self.climb.run()


    def testInit(self):
        """This function is called at the beginning of test mode."""

        pass

    def testPeriodic(self):
        """This function is called periodically during test mode."""

        # pass
        self.arm.enable(False)
        value = (self.normalize(self.controller.getTriggerAxis(0), 0.15) + (self.normalize(self.controller.getTriggerAxis(1), 0.15) * -1))
        # self.front_left_climb.set(value  * 0.47)
        # self.front_right_climb.set(value  * 0.7)
        # self.rear_climb.set(value  * 0.55)
        self.actuator.set(value  * 0.5)
    
    def robotPeriodic(self):
        """This function is called periodically at the end of every robot mode."""

        self.sd.putNumber('drive/front_left', self.front_left.getQuadraturePosition())
        self.sd.putNumber('drive/front_right', self.front_right.getQuadraturePosition())
        self.sd.putNumber('drive/rear_left', self.rear_left.getQuadraturePosition())
        self.sd.putNumber('drive/rear_right', self.rear_right.getQuadraturePosition())

        # self.sd.putBoolean('arm/arm_low_direction', self.arm_low_encoder.getDirection())

        wpilib.SmartDashboard.updateValues()
        wpilib.LiveWindow.updateValues()
        wpilib.shuffleboard.Shuffleboard.update()

    def disabledPeriodic(self):
        """This function is called periodically during disabled mode."""
        pass
        # self.arm.disable()
        # self.arm_low.stopMotor()

    def normalize(self, joystickInput, deadzone):
        """joystickInput should be between -1 and 1, deadzone should be between 0 and 1."""

        if joystickInput > 0:
            if (joystickInput - deadzone) < 0:
                return 0
            else:
                return (joystickInput - deadzone) / (1 - deadzone)
        elif joystickInput < 0:
            if (joystickInput + deadzone) > 0:
                return 0
            else:
                return (joystickInput + deadzone) / (1 - deadzone)
        else:
            return 0
    
    def clamp(self, value: float, low: float, high: float) -> float:
        """Clamps input to maximum and minimum values."""

        return max(low, min(value, high))


if __name__ == "__main__":
    wpilib.run(MyRobot)
