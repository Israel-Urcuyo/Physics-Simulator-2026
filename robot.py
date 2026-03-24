from robot_container import RobotContainer
from sysid import record_drive_logs, save_module_logs
import wpilib
import constants
from commands2 import CommandScheduler


class SubDrive(wpilib.TimedRobot):

    def robotInit(self):
        self.robotcontainer = RobotContainer()
        self.auto_command = None
        self.module_logs = []

    def teleopInit(self):
        if self.auto_command != None:
            self.auto_command.cancel()

        self.robotcontainer.configure_button_bindings()
    
    def autonomousInit(self):
        self.auto_command = self.robotcontainer.get_auto()

        if self.auto_command != None:
            self.auto_command.schedule()

    def robotPeriodic(self):
        self.robotcontainer.update()
        CommandScheduler.getInstance().run()

    def teleopPeriodic(self):
        if constants.sysid_log_enabled:
            if len(self.module_logs) > 100000000:
                self.module_logs.clear()
            self.module_logs.append(record_drive_logs(self.robotcontainer.drive_subsystem))

    def disabledInit(self):
        if constants.sysid_log_enabled:
            # TODO: maybe put the date it was recorded
            save_module_logs("/home/lvuser/log.csv", self.module_logs)
            self.module_logs.clear()
