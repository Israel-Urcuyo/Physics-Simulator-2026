import phoenix6.hardware
import phoenix6.controls
import wpimath.controller
import wpimath.kinematics
import wpimath.trajectory
import wpimath.geometry
import math
import wpilib

from rev import SparkMax
import constants

def zero_enc(value, inital):
    return value - inital

class SwerveModule:

    def get_drive_position(self):
        """Returns position in meters"""
        return -1 * self.drive_motor.get_position().value * constants.drive_velocity_conversion_factor

    def get_drive_velocity(self):
        """Returns velocity in meters per second"""
        return -1 * float(self.drive_motor.get_velocity().value) * constants.drive_velocity_conversion_factor

    def get_turning_position(self):
        """Get offset encoder position in radians"""

        offset_value = (zero_enc(self.absolute_encoder.get(), self.offset))
        
        return wpimath.geometry.Rotation2d(offset_value * 2 * math.pi)

    def __init__(self, drive_motor: int, turning_motor: int, absolute_encoderID: int, offset: int):
        self.offset = offset
        self.drive_motor = phoenix6.hardware.talon_fx.TalonFX(drive_motor)
        self.turning_motor = SparkMax(turning_motor, SparkMax.MotorType.kBrushless)
        self.absolute_encoderID = absolute_encoderID
        self.absolute_encoder = wpilib.AnalogEncoder(absolute_encoderID)

        self.turning_encoder = self.turning_motor.getEncoder()
            
        self.drive_PID_controller = wpimath.controller.PIDController(
            constants.drive_pid_p, constants.drive_pid_i, constants.drive_pid_d)

        # the trapezoidal motion profile is to limit the acceleration and velocity and results in the target velocity looking like a trapezoid
        self.turning_PID_controller = wpimath.controller.ProfiledPIDController(
            constants.turn_pid_p,
            constants.turn_pid_i,
            constants.turn_pid_d,
            wpimath.trajectory.TrapezoidProfile.Constraints(
                constants.turning_max_velocity,
                constants.turning_max_acceleration,
            ),
        )
        self.turning_PID_controller.setTolerance(constants.turn_pid_tolerance)


        self.drive_feedforward = wpimath.controller.SimpleMotorFeedforwardMeters(
            constants.drive_ff_s, constants.drive_ff_v, constants.drive_ff_a)
        self.turn_feedforward = wpimath.controller.SimpleMotorFeedforwardMeters(
            constants.turn_ff_s, constants.turn_ff_v, constants.turn_ff_a
        ) 

        self.turning_PID_controller.enableContinuousInput(-math.pi, math.pi)

    def get_state(self) -> wpimath.kinematics.SwerveModuleState:
        return wpimath.kinematics.SwerveModuleState(
            self.get_drive_velocity(),
            self.get_turning_position()
        )

    def get_position(self) -> wpimath.kinematics.SwerveModulePosition:
        return wpimath.kinematics.SwerveModulePosition(
            self.get_drive_position(),
            self.get_turning_position()
        )

    def set_desired_state(

        self, state: wpimath.kinematics.SwerveModuleState
    ) -> None:
        encoder_rotation = self.get_turning_position()
        drive_velocity = self.get_drive_velocity()
       

        # Optimize the reference state to avoid spinning further than 90 degrees
        wpimath.kinematics.SwerveModuleState.optimize(
            state, encoder_rotation
        )

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving.
        state.speed *= (state.angle - encoder_rotation).cos()

        drive_output = self.drive_PID_controller.calculate(
            drive_velocity, state.speed
        )

        drive_feed_forward = self.drive_feedforward.calculate(state.speed)

        turn_output = self.turning_PID_controller.calculate(
            encoder_rotation.radians(), state.angle.radians()
        )

        turn_feed_forward = self.turn_feedforward.calculate(
            self.turning_PID_controller.getSetpoint().velocity
        )

        self.drive_motor.set_control(
            phoenix6.controls.VoltageOut(-(drive_output + drive_feed_forward))
        )

        self.turning_motor.setVoltage(
           (turn_output + turn_feed_forward)
        )

