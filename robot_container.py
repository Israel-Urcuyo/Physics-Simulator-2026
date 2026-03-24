from photonlibpy.targeting.photonPipelineResult import PhotonPipelineResult
from subsystems.drive_subsystem import DriveSubsystem
from commands2.button import CommandXboxController, CommandJoystick

from wpimath.geometry import Transform3d
from commands.drive_command import DriveCommand
from commands.stop_drive import StopDrive
from pathplannerlib.auto import AutoBuilder
from wpimath.filter import SlewRateLimiter
from photonlibpy import PhotonCamera, PhotonPoseEstimator
from wpilib import SmartDashboard
from wpimath.estimator import SwerveDrive4PoseEstimator
import constants
import wpimath.geometry
import wpilib
from wpimath.geometry import Pose2d
from commands2.instantcommand import InstantCommand
from cscore import CameraServer
import ntcore


robot_to_cam1 = wpimath.geometry.Transform3d(
    wpimath.geometry.Translation3d(0.37465, 0.01905, 0.2032),
    wpimath.geometry.Rotation3d.fromDegrees(0, 0, 0),
)

robot_to_cam2 = wpimath.geometry.Transform3d(
    wpimath.geometry.Translation3d(-0.203, -0.381, 0.2032),
    wpimath.geometry.Rotation3d.fromDegrees(0, 0, -90),
)


class CameraPoseEstimator:
    """
    Wrapper class for using cameras with photonvision and a pose estimator class.
    """

    def __init__(self, camera_name: str, robot_to_cam: Transform3d):
        self.camera = PhotonCamera(camera_name)

        self.estimator = PhotonPoseEstimator(constants.apriltag_layout,  robot_to_cam)
    
    def update_estimator(self, pose_estimator : SwerveDrive4PoseEstimator):
        # cam_est_pose = self.estimator.update()
        results = self.camera.getAllUnreadResults()

        for result in results:
            self.process_result(result, pose_estimator)

    def process_result(self, result : PhotonPipelineResult, pose_estimator : SwerveDrive4PoseEstimator):
        """processes a result and adds it to the pose estimation"""

        cam_est_pose = self.estimator.estimateCoprocMultiTagPose(result)
        
        if (cam_est_pose != None):
            # targets_were_correct = True
            for target in cam_est_pose.targetsUsed:
                if (
                    target.area < 0 or target.getPoseAmbiguity() > 0.2
                ):  # recommended number here is 0.2, maybe increase?
                    # TODO: Don't disqualify all targets if 1 is unreliable maybe?
                    # targets_were_correct = False
                    cam_est_pose.targetsUsed.remove(target)
            if len(cam_est_pose.targetsUsed) != 0:
                pose_estimator.addVisionMeasurement(
                    cam_est_pose.estimatedPose.toPose2d(),
                    cam_est_pose.timestampSeconds,  # what happens if list is empty? TEST THIS
                )
        
class LimelightPoseEst:
    def __init__(self, table_name : str):
        self.table_name = table_name
        pass
    def update_estimator(self, pose_estimator: SwerveDrive4PoseEstimator):
        
        table = ntcore.NetworkTableInstance.getDefault().getTable(self.table_name)
        est_pose = self.networktables_instance = table.getEntry("botpose_wpiblue").getDoubleArray([])
        fiducials = table.getEntry("rawfiducials").getDoubleArray([])
        ambiguity = 1
    
        if (len(fiducials)<6):
            return
        ambiguity = fiducials[6]

        if(ambiguity>.4):
            return
        if(len(est_pose)<7):
            #print("list not ready!")
            return
        if(est_pose[7]==0):
            return
        
        


        pose = wpimath.geometry.Pose2d(wpimath.geometry.Translation2d(est_pose[0],est_pose[1]),wpimath.geometry.Rotation2d.fromDegrees(est_pose[5]))
        # print(self.table_name + " ", pose)
  
        pose_estimator.addVisionMeasurement(pose, wpilib.Timer.getFPGATimestamp()-(est_pose[6]/1000.0))# Are these array positions correct?
        # print(est_pose)

# deadzone should be lower for good controllers
def deadzone(input: float, deadzone=0.12):
    if (input < deadzone and input > 0) or (input > -deadzone and input < 0):
        return 0
    return input

class RobotContainer:

    def __init__(self):
        #camera = CameraServer.startAutomaticCapture()
        self.drive_subsystem = DriveSubsystem()
        self.drive_controller = CommandXboxController(0)
        # how we get the paths to run in teleop (operator keypad)
        self.path_controller = CommandJoystick(2)
        # self.path_controller = CommandXboxController(2)

        self.field = wpilib.Field2d()

        self.networktables_instance = ntcore.NetworkTableInstance.getDefault()
        # self.keypad_table = self.networktables_instance.getTable("keyin")
        # self.keypad_pressed_keys = []

        #Rate of Change
        #Rate of Change
        
        self.filterY = SlewRateLimiter(constants.roc)
        self.filterX = SlewRateLimiter(constants.roc)
        self.filterRY= SlewRateLimiter(constants.roc) 

        self.auto_chooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("auto chooser", self.auto_chooser)

        self.setup_camera()
        self.configure_button_bindings()

    def setup_camera(self):
        ###############
        # CAMERA CODE #
        ###############
        #self.cameras = [CameraPoseEstimator(constants.camera1, robot_to_cam1), CameraPoseEstimator(constants.camera2, robot_to_cam2)]
        self.cameras = [LimelightPoseEst("limelight"), LimelightPoseEst("limelight-threea")]
    def update(self):     
        self.update_pose_est()

    def update_pose_est(self):

        for camera in self.cameras:
            camera.update_estimator(self.drive_subsystem.pose_est)


        self.drive_subsystem.update_odometry()

        self.field.setRobotPose(self.drive_subsystem.get_pose())

        wpilib.SmartDashboard.putData(self.field)

    def configure_button_bindings(self):

        # if wpilib.DriverStation.getAlliance() != wpilib.DriverStation.Alliance.kRed:

        #     self.drive_subsystem.setDefaultCommand(
        #         DriveCommand(
        #             lambda: -(
        #                 ((deadzone(self.drive_controller.getLeftY())))
        #                 * constants.drivetrain_max_speed
        #                 * constants.drivetrain_linear_speed_percentage
        #             ),
        #             lambda: -(
        #                 ((deadzone(self.drive_controller.getLeftX())))
        #                 * constants.drivetrain_max_speed
        #                 * constants.drivetrain_linear_speed_percentage
        #             ),
        #             lambda: -((deadzone(self.drive_controller.getRightX())))
        #             * constants.drivetrain_angular_speed_factor,
        #             True,
        #             False,
        #             self.drive_subsystem,
        #         )
        #     )
        # else:
        #     self.drive_subsystem.setDefaultCommand(
        #         DriveCommand(
        #             lambda: (
        #                 ((deadzone(self.drive_controller.getLeftY())))
        #                 * constants.drivetrain_max_speed
        #                 * constants.drivetrain_linear_speed_percentage
        #             ),
        #             lambda: (
        #                 ((deadzone(self.drive_controller.getLeftX())))
        #                 * constants.drivetrain_max_speed
        #                 * constants.drivetrain_linear_speed_percentage
        #             ),
        #             lambda: -((deadzone(self.drive_controller.getRightX())))
        #             * constants.drivetrain_angular_speed_factor,
        #             True,
        #             False,
        #             self.drive_subsystem,
        #         )
        #     )

        # for testing, this will overwrite the alliance specific DriveCommands
        self.drive_subsystem.setDefaultCommand(
            DriveCommand(
                lambda: -(
                    ((deadzone(self.drive_controller.getLeftY())))
                    * constants.drivetrain_max_speed
                    * constants.drivetrain_linear_speed_percentage
                ),
                lambda: -(
                    ((deadzone(self.drive_controller.getLeftX())))
                    * constants.drivetrain_max_speed
                    * constants.drivetrain_linear_speed_percentage
                ),
                lambda: -((deadzone(self.drive_controller.getRightX())))
                * constants.drivetrain_angular_speed_factor,
                True,
                lambda : self.drive_controller.a().getAsBoolean(),
                self.drive_subsystem,
            )
        )

        def get_x_speed():
            value = 0
            if self.drive_controller.povDown().getAsBoolean():
                value -= 0.2
            elif self.drive_controller.povUp().getAsBoolean():
                value += 0.2
            return value

        robot_frame_drive_command = DriveCommand(
            x_speed=get_x_speed,
            y_speed=lambda: (
                (deadzone(self.drive_controller.getLeftTriggerAxis()) * 0.6)
                - (deadzone(self.drive_controller.getRightTriggerAxis()) * 0.6)
            ),
            rotation_speed=lambda: 0,
            field_relative=False,
            align_to_hub=lambda : self.drive_controller.a().getAsBoolean(),
            subsystem=self.drive_subsystem,
        )

        self.drive_controller.b().onTrue(InstantCommand(self.drive_subsystem.pose_est.resetPose(constants.inital_pose)))

        # self.drive_controller.rightTrigger(threshold=0.01).whileTrue(
        #    robot_frame_drive_command
        # )

        # self.drive_controller.leftTrigger(threshold=0.01).whileTrue(
        #     robot_frame_drive_command
        # )

        # self.drive_controller.povDown().whileTrue(
        # robot_frame_drive_command
        # )

        # self.drive_controller.povUp().whileTrue(
        #  robot_frame_drive_command
        # )

       

  
    def get_auto(self):
       return self.drive_subsystem.get_auto("testauto2").andThen(StopDrive(self.drive_subsystem))
