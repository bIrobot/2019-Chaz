import wpilib
from magicbot import tunable
import ctre

import numpy as np


ENCODER_TICKS_PER_DEGREE = 2048 / 360
LOW_ARM_WEIGHT = 20
MIDDLE_ARM_WEIGHT = 3
CONE_WEIGHT = 2.5 + 2.2
LOW_MOMENT_ARM = 11
MIDDLE_MOMENT_ARM = 20


class Arm:
    '''
        Robot Arm
    '''

    arm_low: ctre.WPI_VictorSPX
    arm_middle: ctre.WPI_VictorSPX
    arm_high: ctre.WPI_VictorSPX
    arm_low_encoder: wpilib.Encoder
    arm_middle_encoder: wpilib.Encoder
    arm_high_encoder: wpilib.Encoder
    arm_low_switch: wpilib.DigitalInput
    arm_middle_switch: wpilib.DigitalInput
    arm_high_switch: wpilib.DigitalInput

    low_p = tunable(default=0.0015)
    low_i = tunable(default=0.00015)
    low_d = tunable(default=0.0015)

    middle_p = tunable(default=0.0015)
    middle_i = tunable(default=0.0001)
    middle_d = tunable(default=0.0002)#-0.003

    high_p = tunable(default=0.002)
    high_i = tunable(default=0.0003)
    high_d = tunable(default=0.001)
    
    low_f = tunable(default=1)
    middle_f = tunable(default=1)
    low_angle = tunable(default=0)
    middle_angle = tunable(default=0)
    high_angle = tunable(default=0)
    low_range = tunable(default=0)
    middle_range = tunable(default=0)
    high_range = tunable(default=0)


    def __init__(self):
        self.low_joint_angle = 0
        self.middle_joint_angle = 0

        self.x = 0
        self.y = 0
        self.theta = 0
        self.low_torque = 0
        self.middle_torque = 0

        self.high_offset = 0

    def setup(self):
        self.low_pid = PIDController(self.low_p, self.low_i, self.low_d, 0, self.arm_low_encoder, self.arm_low)
        self.middle_pid = PIDController(self.middle_p, self.middle_i, self.middle_d, 0, self.arm_middle_encoder, self.arm_middle)
        self.high_pid = PIDController(self.high_p, self.high_i, self.high_d, 0, self.arm_high_encoder, self.arm_high)
        
        self.low_pid.setOutputRange(-1, 1)
        self.middle_pid.setOutputRange(-0.5, 0.5)
        self.high_pid.setOutputRange(-0.5, 0.5)

        self.low_pid.setInputRange(-95 * ENCODER_TICKS_PER_DEGREE, 85 * ENCODER_TICKS_PER_DEGREE)
        self.middle_pid.setInputRange(-290 * ENCODER_TICKS_PER_DEGREE, 0 * ENCODER_TICKS_PER_DEGREE)
        self.high_pid.setInputRange(-15 * ENCODER_TICKS_PER_DEGREE, 120 * ENCODER_TICKS_PER_DEGREE)

        # self.low_pid.setDeadzone(15, 50)
        self.low_pid.setDeadzone(0, 0)
        # self.middle_pid.setDeadzone(20, 50)
        self.middle_pid.setDeadzone(0, 0)
        self.high_pid.setDeadzone(5, 25)
        # self.high_pid.setDeadzone(0, 0)

        self.low_pid.enable()
        self.middle_pid.enable()
        self.high_pid.enable()
        
        self.arm_low_switch_last_value = self.arm_low_switch.get()
        self.arm_middle_switch_last_value = self.arm_middle_switch.get()
        self.arm_high_switch_last_value = self.arm_high_switch.get()

        self.low_widget = (
            wpilib.shuffleboard.Shuffleboard.getTab("My Tab")
            .add(self.low_pid, title="Low PID")
            .withWidget(wpilib.shuffleboard.BuiltInWidgets.kPIDController)
        )

        self.middle_widget = (
            wpilib.shuffleboard.Shuffleboard.getTab("My Tab")
            .add(self.middle_pid, title="Middle PID")
            .withWidget(wpilib.shuffleboard.BuiltInWidgets.kPIDController)
        )

        self.high_widget = (
            wpilib.shuffleboard.Shuffleboard.getTab("My Tab")
            .add(self.high_pid, title="High PID")
            .withWidget(wpilib.shuffleboard.BuiltInWidgets.kPIDController)
        )
    
    def set_offset(self, offset):
        self.high_offset = offset

    def set_arm_setpoint(self, low, middle, high):
        # self.low_pid.setSetpoint(low * ENCODER_TICKS_PER_DEGREE)
        # self.middle_pid.setSetpoint(middle * ENCODER_TICKS_PER_DEGREE)
        # self.high_pid.setSetpoint(high * ENCODER_TICKS_PER_DEGREE)
        self.low_pid.setSetpoint((low * -1 + 61.5) * ENCODER_TICKS_PER_DEGREE)
        self.middle_pid.setSetpoint((middle * -1 - 153) * ENCODER_TICKS_PER_DEGREE)
        if (self.middle_arm_angle < 90):
            self.high_pid.setSetpoint(((self.middle_arm_angle + 80) + self.high_offset) * ENCODER_TICKS_PER_DEGREE)
        else:
            self.high_pid.setSetpoint(((self.middle_arm_angle - 100) + self.high_offset) * ENCODER_TICKS_PER_DEGREE)

    def disable(self):
        self.low_pid.disable()
        self.middle_pid.disable()
        self.high_pid.disable()
    
    def enable(self):
        self.low_pid.enable()
        self.middle_pid.enable()
        self.high_pid.enable()

    def execute(self):
        if self.arm_low_switch_last_value is False and self.arm_low_switch.get() is True and self.arm_low_encoder.getDirection() is False:
            self.arm_low_encoder.reset()
        
        if self.arm_middle_switch_last_value is False and self.arm_middle_switch.get() is True and self.arm_middle_encoder.getDirection() is False:
            self.arm_middle_encoder.reset()
        
        if self.arm_high_switch_last_value is False and self.arm_high_switch.get() is True and self.arm_high_encoder.getDirection() is True:
            self.arm_high_encoder.reset()

        self.arm_low_switch_last_value = self.arm_low_switch.get()
        self.arm_middle_switch_last_value = self.arm_middle_switch.get()
        self.arm_high_switch_last_value = self.arm_high_switch.get()

        self.low_joint_angle = self.arm_low_encoder.get() * (360 / 2048) * -1 + 61.5
        self.middle_joint_angle = self.arm_middle_encoder.get() * (360 / 2048) * -1 - 153
        self.high_joint_angle = self.arm_high_encoder.get() * (360 / 2048) * -1 + (153 - 61.5)
        low_arm_angle = self.low_joint_angle
        self.middle_arm_angle = low_arm_angle + self.middle_joint_angle
        cone_angle = self.middle_arm_angle + self.high_joint_angle
        self.low_pid.setF(self.low_feedforward(self.low_joint_angle, self.middle_joint_angle, LOW_ARM_WEIGHT, MIDDLE_ARM_WEIGHT + CONE_WEIGHT, LOW_MOMENT_ARM, MIDDLE_MOMENT_ARM))
        self.middle_pid.setF(((MIDDLE_ARM_WEIGHT + CONE_WEIGHT - 2.2) * MIDDLE_MOMENT_ARM * np.cos(self.middle_arm_angle * np.pi / 180)) / (12.48 * 100) * -1)

        self.low_f = self.low_pid.getF()
        self.middle_f = self.middle_pid.getF()
        self.low_angle = low_arm_angle
        self.middle_angle = self.middle_joint_angle
        self.high_angle = cone_angle

        # self.low_range = 1 / max(1, min(abs(self.arm_low_encoder.getRate()) / 150, 8))
        self.low_range = 1
        self.low_pid.setOutputRange(-1 * self.low_range, 1 * self.low_range)
        # self.middle_range = 1 / max(1, min(abs(self.arm_low_encoder.getRate()) / 20000, 8))
        self.middle_range = 1
        self.middle_pid.setOutputRange(-1 * self.middle_range, 1 * self.middle_range)
        self.high_range = 1 / max(1, min(abs(self.arm_low_encoder.getRate()) / 10, 10))
        self.high_pid.setOutputRange(-0.5 * self.high_range, 0.5 * self.high_range)

    def low_feedforward(self, low_angle, middle_angle, low_weight, middle_weight, low_length, middle_length):
        low_angle = low_angle * np.pi / 180
        middle_angle = middle_angle * np.pi / 180

        self.low_torque = low_weight * low_length * np.cos(low_angle)

        self.x = middle_length * np.cos(low_angle + middle_angle) + 28 * np.cos(low_angle)
        self.y = middle_length * np.sin(low_angle + middle_angle) + 28 * np.sin(low_angle)
        self.theta = float(np.arctan2(self.y, self.x))
        self.middle_torque = middle_weight * np.sqrt(self.x**2 + self.y**2) * np.cos(self.theta)

        return (self.low_torque + self.middle_torque) / (21.33 * 80) * -1


class PIDController(wpilib.PIDController):

    deadzone = 0

    def _calculate(self) -> None:
        """Read the input, calculate the output accordingly, and write to the
        output.  This should only be called by the PIDTask and is created
        during initialization."""
        if self.origSource is None or self.pidOutput is None:
            return

        with self.mutex:
            enabled = self.enabled  # take snapshot of these values...

        if enabled:
            feedForward = self.calculateFeedForward()

            with self.mutex:
                input = self.pidInput.pidGet()
                pidSourceType = self.pidInput.getPIDSourceType()
                P = self.P
                I = self.I
                D = self.D
                minimumOutput = self.minimumOutput
                maximumOutput = self.maximumOutput
                prevError = self.prevError
                error = self.getContinuousError(self.setpoint - input)
                totalError = self.totalError

            # start

            if pidSourceType == self.PIDSourceType.kRate:
                if P != 0:
                    totalError = self.clamp(
                        totalError + error, minimumOutput / P, maximumOutput / P
                    )

                result = P * totalError + D * error + feedForward

            else:
                if I != 0:
                    totalError = self.clamp(
                        totalError + error, minimumOutput / I, maximumOutput / I
                    )

                result = (
                    P * error + I * totalError + D * (error - prevError) + feedForward
                )

            result = self.clamp(result, minimumOutput, maximumOutput)

            with self.pidWriteMutex:
                with self.mutex:
                    enabled = self.enabled
                if enabled:
                    self.calculateOutput(result)

            with self.mutex:
                self.prevError = error
                self.error = error
                self.totalError = totalError
                self.result = result

    def calculateFeedForward(self) -> float:
        return self.F
    
    def setDeadzone(self, low, high):
        self.low_deadzone = low
        self.high_deadzone = high

    def calculateOutput(self, input):
        if self.F < 0:
            if (self.getSetpoint() - self.pidInput.pidGet()) > self.low_deadzone and (self.getSetpoint() - self.pidInput.pidGet()) < self.high_deadzone:
                self.pidOutput(0)
            else:
                self.pidOutput(input)
        else:
            if (self.getSetpoint() - self.pidInput.pidGet()) < self.low_deadzone * -1 and (self.getSetpoint() - self.pidInput.pidGet()) > self.high_deadzone * -1:
                self.pidOutput(0)
            else:
                self.pidOutput(input)
