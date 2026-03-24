import phoenix6.unmanaged
from wpilib import Field2d, SmartDashboard
from wpimath.geometry import Transform2d, Rotation2d
from pyfrc.physics.core import PhysicsInterface
from robot import SubDrive
from swervemodel import SwerveModel


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: SubDrive):
        return
        physics_controller.move_robot(Transform2d(5.0, 5.0, Rotation2d(0)))

        self.physics_controller = physics_controller
        self.robot = robot
        self.gyro = robot.robotcontainer.drive_subsystem.gyro

        drivetrain = robot.robotcontainer.drive_subsystem

        # Use .theory() method (sensible default)
        self.swerve_model = SwerveModel.theory(
            # 8 motors
            fl_drive_motor=drivetrain.front_left.drive_motor,
            fl_turn_motor=drivetrain.front_left.turning_motor,
            fr_drive_motor=drivetrain.front_right.drive_motor,
            fr_turn_motor=drivetrain.front_right.turning_motor,
            bl_drive_motor=drivetrain.back_left.drive_motor,
            bl_turn_motor=drivetrain.back_left.turning_motor,
            br_drive_motor=drivetrain.back_right.drive_motor,
            br_turn_motor=drivetrain.back_right.turning_motor,
            # Robot physics
            robot_mass=38.555,
            robot_width=0.749,
            robot_length=0.749,
            wheel_radius=0.051,
            drive_gearing=6.75,  # Your actual drive gearing
            turn_gearing=12.8,  # Your actual turn gearing
        )

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

    def update_sim(self, now: float, tm_diff: float):
        return
        phoenix6.unmanaged.feed_enable(100)

        transform = self.swerve_model.calculate(tm_diff)
        self.physics_controller.move_robot(transform)
        self.gyro.setYaw(-transform.rotation().degrees())

        pose = self.physics_controller.field.getRobotPose()
        self.field.setRobotPose(pose)


def physicsInit(physics_controller):
    robot = SubDrive()
    return PhysicsEngine(physics_controller, robot)


def update_sim(physics_engine, now, tm_diff):
    physics_engine.update_sim(now, tm_diff)
