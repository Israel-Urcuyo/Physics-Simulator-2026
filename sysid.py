from subsystems.drive_subsystem import DriveSubsystem
from commands2 import Command
import phoenix6.controls
import constants
from subsystems.drive_subsystem import DriveSubsystem
from subsystems.swerve_module import SwerveModule
from commands2 import Command
import phoenix6.controls
import constants
from wpimath.units import volts, meters_per_second, meters, radians
from dataclasses import dataclass
from typing import List


import csv

@dataclass
class SysidModuleLog:
    drive_voltage : volts
    turn_voltage : volts

    drive_speed : meters_per_second

    drive_position : meters
    turn_position : radians
    id : int




def record_module_log(module : SwerveModule) -> SysidModuleLog:
    return SysidModuleLog(
        drive_voltage=(module.drive_motor.get_motor_voltage().value),
        # this might be the wrong value
        turn_voltage=(module.turning_motor.getBusVoltage() * module.turning_motor.getAppliedOutput()),

        drive_speed=(module.get_drive_velocity()),
        turn_position=(module.get_turning_position().radians()),

        drive_position=(module.get_drive_position()),
        
        id = module.absolute_encoderID
    )

def record_drive_logs(subsystem : DriveSubsystem) -> List[SysidModuleLog]:
    return [
        record_module_log(subsystem.front_left),
        record_module_log(subsystem.front_right),
        record_module_log(subsystem.back_left),
        record_module_log(subsystem.back_right),
    ]

def save_module_logs(filename : str, logs : List[List[SysidModuleLog]]):
    
    # TODO: this will create a file that takes up space on the roborio, maybe remove it at some point

    csvfile = None
    try:
        csvfile = open (filename, "w")

    except FileNotFoundError:
        # TODO: this might result in issues where the log file is not created automatically on the roborio if it is deleted, but it makes the tests pass
        print("THE LOG FILE DOES NOT EXIST AND NEEDS TO BE CREATED")
        return
    
    writer = csv.writer(csvfile, delimiter=',',
                    quoting=csv.QUOTE_MINIMAL)

    for log in logs:
        for module in log:
            writer.writerow([module.drive_speed, module.drive_voltage, module.drive_position, module.turn_position, module.turn_voltage, module.id])

            # row = []
            # for module in log:
            #     row.append([module.drive_speed, module.drive_voltage, module.drive_position, module.turn_position, module.turn_voltage])
            # writer.writerow(row)


def set_all_motors_drive(subsystem : DriveSubsystem, voltage):
    subsystem.front_left.drive_motor.set_control(phoenix6.controls.VoltageOut(voltage))
    subsystem.front_right.drive_motor.set_control(phoenix6.controls.VoltageOut(voltage))
    subsystem.back_left.drive_motor.set_control(phoenix6.controls.VoltageOut(voltage))
    subsystem.back_right.drive_motor.set_control(phoenix6.controls.VoltageOut(voltage))

def set_all_motors_turn(subsystem : DriveSubsystem, voltage):
    subsystem.front_left.turning_motor.setVoltage((voltage))
    subsystem.front_right.turning_motor.setVoltage((voltage))
    subsystem.back_left.turning_motor.setVoltage((voltage))
    subsystem.back_right.turning_motor.setVoltage((voltage))

class SysidStaticCommandDrive(Command):

    def __init__(self, subsystem : DriveSubsystem):
        super().__init__()

        self.subsystem = subsystem
        self.starting_pos = self.subsystem.front_left.drive_motor.get_position().value
        self.currentVoltage = 0
        self.done = False

    def execute(self):
        super().execute()
        print(self.subsystem.front_left.drive_motor.get_position().value)
        if abs(self.subsystem.front_left.drive_motor.get_position().value) > self.starting_pos + 0.01:
            print("drive kS:" + str(self.currentVoltage))
            self.done = True

        else:
            self.currentVoltage += 0.01
            print("voltage: " + str(self.currentVoltage))
            set_all_motors_drive(self.subsystem, self.currentVoltage)

    def isFinished(self):
        return self.done
    
class SysidVelocityCommandDrive(Command):
    def __init__(self, subsystem : DriveSubsystem, max_voltage, max_velocity = constants.drivetrain_max_speed):
        """
        velocity is in meters per second
        this could run forever if it never reaches the max speed
        it could also be bad if the wheels are not facing the same directon
        """        
        super().__init__()

        self.subsystem = subsystem
        self.done = False
        self.max_velocity = max_velocity
        self.max_voltage = max_voltage


    def execute(self):
        self.subsystem.front_left.drive_motor.get_motor_voltage()
        set_all_motors_drive(self.subsystem, self.max_voltage)
        speed = self.subsystem.get_speed()

        if speed.vy > self.max_velocity:
            print("drive kV:" + str(self.max_voltage / speed.vy))
            self.done = True

        if speed.vx > self.max_velocity:
            print("drive kV:" + str(self.max_voltage / speed.vx))
            self.done = True

    def isFinished(self):
        return self.done

class SysidStaticCommandTurn(Command):

    def __init__(self, subsystem : DriveSubsystem):
        super().__init__()

        self.subsystem = subsystem
        self.current_voltage = 0
        self.done = False

    def execute(self):
        super().execute()

        if self.subsystem.front_left.get_turning_position().radians() > 1:
            print("turn kS:" + str(self.current_voltage))
            self.done = True

        else:
            self.current_voltage += 0.01
            set_all_motors_turn(self.subsystem, self.current_voltage)

    def isFinished(self) -> bool:
        return self.done
    
class SysidVelocityCommandTurn(Command):

    def __init__(self, subsystem : DriveSubsystem, max_voltage, max_velocity = constants.turning_max_velocity):
        """
        velocity is in radians per second
        this could run forever if it never reaches the max speed
        """        
        super().__init__()

        self.subsystem = subsystem
        self.done = False
        self.max_velocity = max_velocity
        self.max_voltage = max_voltage

        self.starting_pos = self.subsystem.front_left.drive_motor.get_position()

    def execute(self):
        super().execute()
        set_all_motors_turn(self.subsystem, self.max_voltage)

        velocity = self.subsystem.get_speed().omega
        if velocity > self.max_velocity:
            print("turn kV: " + str(self.max_voltage / velocity))
            self.done = True

    def isFinished(self):
        return self.done
