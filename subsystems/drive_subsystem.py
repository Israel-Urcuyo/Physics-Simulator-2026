import commands2
import wpimath.kinematics
from wpimath.kinematics import SwerveModulePosition, ChassisSpeeds
from wpimath.geometry import Translation2d, Rotation2d, Pose3d, Pose2d
from wpilib import DriverStation, RobotBase
from phoenix5.sensors import PigeonIMU
from wpimath.units import degreesToRadians, meters_per_second, radians_per_second, degrees
from subsystems.swerve_module import SwerveModule
import constants
import wpimath.estimator
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import PIDConstants, RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.util import DriveFeedforwards
from pathplannerlib.path import PathPlannerPath




class DriveSubsystem(commands2.Subsystem):

    def get_speed(self):
        return self.kinematics.toChassisSpeeds((
            self.front_left.get_state(),
            self.front_right.get_state(),
            self.back_left.get_state(),
            self.back_right.get_state(),

        ))
    
    def get_pose(self):
        return self.pose_est.getEstimatedPosition()

    def reset_position(self, pose):
        self.pose_est.resetPose(pose)

    def __init__(self):
        self.gyro = PigeonIMU(30)
        self.gyro.setYaw(0)

        self.front_left = SwerveModule(6, 4, 0, constants.front_left_offset)
        self.front_right = SwerveModule(7, 3, 1, constants.front_right_offset)

        self.back_left = SwerveModule(8, 2, 2, constants.back_left_offset)
        self.back_right = SwerveModule(9, 5, 3, constants.back_right_offset) 


        self.front_left_location = constants.front_left_gyro_offset
        self.front_right_location = constants.front_right_gyro_offset
        self.back_left_location = constants.back_left_gyro_offset
        self.back_right_location = constants.back_right_gyro_offset

        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.front_left_location,
            self.front_right_location,
            self.back_left_location,
            self.back_right_location,
        )

       

        self.pose_est = wpimath.estimator.SwerveDrive4PoseEstimator(
            self.kinematics,
            self.get_gyro_rotation2d(),
            (
                self.front_left.get_position(),
                self.front_right.get_position(),
                self.back_left.get_position(),
                self.back_right.get_position()
            ),
            constants.inital_pose,
        )

        self.pose_est.setVisionMeasurementStdDevs((0.9, 0.9, 1.8))
        
        # # broken for wpilib version 2024
        config = RobotConfig.fromGUISettings()

        # Configure the AutoBuilder last
        AutoBuilder.configure(
            self.get_pose,
            self.reset_position, # Method to reset odometry (will be called if your auto has a starting pose)
            self.get_speed, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.pathplanner_drive, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also outputs individual module feedforwards
            PPHolonomicDriveController( # PPHolonomicController is the built in path following controller for holonomic drive trains
                PIDConstants(3, 0, 0.05), # Translation PID constants
                PIDConstants(3.5, 0, 0.8) # Rotation PID constants
                
            ),
            config, # The robot configuration
            should_flip_path, # Supplier to control path flipping based on alliance color
            self
        )

    def pathplanner_drive(self, speeds : ChassisSpeeds, feedforwards: DriveFeedforwards):
       
       self.drive(speeds.vx, speeds.vy, speeds.omega, False, 0.02)


    def get_heading(self) -> degrees:
        return self.get_pose().rotation().degrees()    


    def get_gyro_rotation2d(self) -> Rotation2d:
        # self.gyro.setYaw(self.gyro.getYaw() % 360)
        return Rotation2d(degreesToRadians(self.gyro.getYaw()))


    def drive(
        self,
        x_speed: meters_per_second,
        y_speed: meters_per_second,
        rot: radians_per_second,
        field_relative: bool,
        period_seconds: float = 0.02
    ) -> None:
        """
        x and y speeds are in meters per second
        """

        swerve_module_states = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds.discretize(
                (
                    wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                        x_speed, y_speed, rot, (self.get_pose()).rotation()
                    )
                    if field_relative
                    else wpimath.kinematics.ChassisSpeeds(x_speed, y_speed, rot)
                ),
                period_seconds,
            )
        )
        swerve_module_states = wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerve_module_states, constants.drivetrain_max_speed
        )


        self.front_left.set_desired_state(swerve_module_states[0])
        self.front_right.set_desired_state(swerve_module_states[1])
        self.back_left.set_desired_state(swerve_module_states[2])
        self.back_right.set_desired_state(swerve_module_states[3])


    def add_vision_pose_estimate(self, pose: Pose3d, timestamp: float):
        self.pose_est.addVisionMeasurement(pose.toPose2d(), timestamp)

    def update_odometry(self):

        self.pose_est.update(
            self.get_gyro_rotation2d(),
             (
                self.front_left.get_position(),
                self.front_right.get_position(),
                self.back_left.get_position(),
                self.back_right.get_position()
            ),
        )
        

    def get_auto(self, name : str):
        return AutoBuilder.buildAuto(name)
    
    def pathfind_to_path(self, path_name, constraints = constants.pathfinding_constraints):
        return AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile(path_name), constraints)


def should_flip_path():
        if RobotBase.isReal():
            return DriverStation.getAlliance() == DriverStation.Alliance.kRed
        
        return False
