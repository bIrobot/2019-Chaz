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
import components.arm_controller
import components.cone
import components.drivetrain


class MyRobot(magicbot.MagicRobot):
    drivetrain: components.drivetrain.Drivetrain
    arm: components.arm.Arm
    arm_controller: components.arm_controller.ArmController
    cone: components.cone.Cone

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
        self.arm_low = ctre.WPI_VictorSPX(5)
        self.arm_middle = ctre.WPI_VictorSPX(6)
        self.arm_high = ctre.WPI_VictorSPX(7)
        self.cone_motor = ctre.WPI_VictorSPX(8)
        self.hatch_grabber = ctre.WPI_VictorSPX(9)

        self.arm_low.setInverted(True)
        self.arm_high.setInverted(True)
        
        # Initialize encoders
        self.arm_low_encoder = wpilib.Encoder(0, 1)
        self.arm_middle_encoder = wpilib.Encoder(2, 3)
        self.arm_high_encoder = wpilib.Encoder(4, 5)
        self.cone_motor_encoder = wpilib.Encoder(6, 7)

        # Initialize switches and hall-effect sensors
        self.cone_switch = wpilib.DigitalInput(8)
        self.hatch_grabber_switch = wpilib.DigitalInput(9)
        self.arm_low_switch = wpilib.DigitalInput(10)
        self.arm_middle_switch = wpilib.DigitalInput(11)
        self.arm_high_switch = wpilib.DigitalInput(12)

        # Communicate w/navX MXP via the MXP SPI Bus.
        # - Alternatively, use the i2c bus.
        # See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details

        # self.navxSensor = navx.ahrs.AHRS.create_spi()
        self.navxSensor = navx.ahrs.AHRS.create_i2c()

        self.arm_x = 12
        self.arm_y = 0

    def teleopInit(self):
        """This function is called at the beginning of teleoperated mode."""

        # self.arm.enable()
        self.arm.execute()
    
    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""

        leftXAxis = self.normalize(self.controller.getX(0), 0.15)
        leftYAxis = self.normalize(self.controller.getY(0), 0.15)
        rotation = self.normalize(self.controller.getX(1), 0.15)

        self.drivetrain.move(leftYAxis, leftXAxis, rotation)

        if self.controller.getYButton():
            self.cone.release()

        # self.arm_x = self.clamp(self.arm_x + self.normalize(self.controller.getX(0), 0.2), -10, 40)
        # self.arm_y = self.clamp(self.arm_y + self.normalize(self.controller.getY(1) * -1, 0.2), -5, 56)

        # self.arm_controller.set_arm_position(self.arm_x, self.arm_y)

    def testInit(self):
        """This function is called at the beginning of test mode."""

        self.arm.enable()
        # self.arm.disable()
        self.arm.execute()

    def testPeriodic(self):
        """This function is called periodically during test mode."""

        if self.controller.getYButton():
            self.cone.release()

        if self.gamepad.getL1Button():
            self.arm_x = 30
            self.arm_y = -0.125
        elif self.gamepad.getXButton():
            self.arm_x = 30
            self.arm_y = 27.875
        elif self.gamepad.getYButton():
            self.arm_x = 0
            self.arm_y = 55.875
        elif self.gamepad.getL2Button():
            self.arm_x = 12
            self.arm_y = 0

        self.arm_controller.set_arm_position(self.arm_x, self.arm_y)

        self.arm_controller.execute()
        self.arm.execute()
        self.cone.execute()
    
    def robotPeriodic(self):
        """This function is called periodically at the end of every robot mode."""

        self.sd.putNumber('drive/front_left', self.front_left.getQuadraturePosition())
        self.sd.putNumber('drive/front_right', self.front_right.getQuadraturePosition())
        self.sd.putNumber('drive/rear_left', self.rear_left.getQuadraturePosition())
        self.sd.putNumber('drive/rear_right', self.rear_right.getQuadraturePosition())

        self.sd.putBoolean('arm/arm_low_direction', self.arm_low_encoder.getDirection())

        wpilib.SmartDashboard.updateValues()
        wpilib.LiveWindow.updateValues()
        wpilib.shuffleboard.Shuffleboard.update()

    def disabledPeriodic(self):
        """This function is called periodically during disabled mode."""

        self.arm.disable()
        self.arm_low.stopMotor()
        self.arm_middle.stopMotor()
        self.arm_high.stopMotor()

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
