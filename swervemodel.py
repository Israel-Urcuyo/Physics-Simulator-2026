import math
import phoenix6
import rev
from wpimath.geometry import Transform2d, Rotation2d



# Default parameters taken from FRC TankModel architecture
DEFAULT_DRIVE_PARAMS = {'kv': 0.8, 'ka': 0.15, 'vintercept': 1.3}
DEFAULT_TURN_PARAMS = {'kv': 0.5, 'ka': 0.1, 'vintercept': 1.3}

class SimpleTalonFXMotorSim:
    def __init__(self, motor: phoenix6.hardware.TalonFX, **params):
        self.sim_state = motor.sim_state
        self.sim_state.set_supply_voltage(12.0)
        
        # TankModel state
        self.acceleration = 0.0  # ft/s^2
        self.velocity = 0.0      # ft/s
        self.position = 0.0      # ft
        self.last_torque = 0.0   # Nm (for force calculations)
        
        # TankModel parameters
        self._nominalVoltage = 12.0
        self._vintercept = params.get('vintercept', DEFAULT_DRIVE_PARAMS['vintercept'])
        self._kv = params.get('kv', DEFAULT_DRIVE_PARAMS['kv'])
        self._ka = params.get('ka', DEFAULT_DRIVE_PARAMS['ka'])

    def compute(self, motor_pct: float, tm_diff: float) -> float:
        """TankModel MotorModel.compute() - exact copy"""
        appliedVoltage = self._nominalVoltage * motor_pct
        appliedVoltage = math.copysign(
            max(abs(appliedVoltage) - self._vintercept, 0), appliedVoltage
        )

        a0 = self.acceleration
        v0 = self.velocity
        v1 = v0 + a0 * tm_diff
        a1 = (appliedVoltage - self._kv * v1) / self._ka
        v1 = v0 + (a0 + a1) * 0.5 * tm_diff
        a1 = (appliedVoltage - self._kv * v1) / self._ka
        self.position += (v0 + v1) * 0.5 * tm_diff

        self.velocity = v1
        self.acceleration = a1
        return self.velocity

    def update(self, dt: float, wheel_radius_m: float, max_wheel_torque: float):
        """Update physics and calculate torque"""
        motor_power = self.sim_state.motor_voltage / self._nominalVoltage
        linear_velocity = self.compute(motor_power, dt)
        
        # Calculate torque (simple model)
        effective_voltage = abs(motor_power * 12.0) - self._vintercept
        if effective_voltage > 0:
            self.last_torque = (effective_voltage / 12.0) * max_wheel_torque
            if motor_power < 0:
                self.last_torque = -self.last_torque
        else:
            self.last_torque = 0.0
        
        # Update Phoenix sim (convert ft/s to rad/s)
        wheel_radius_ft = wheel_radius_m * 3.28084
        angular_velocity = linear_velocity / wheel_radius_ft
        angular_position = self.position / wheel_radius_ft
        self.sim_state.set_rotor_velocity(angular_velocity)
        self.sim_state.set_rotor_position(angular_position)


class SparkMaxTurningSim:
    def __init__(self, motor: rev.SparkMax, **params):
        self.motor = motor
        self.sim_encoder = rev.SparkRelativeEncoderSim(motor)
        
        # Angular state
        self.acceleration = 0.0  # rad/s^2
        self.velocity = 0.0      # rad/s
        self.position = 0.0      # rad
        
        # TankModel parameters for angular motion
        self._nominalVoltage = 12.0
        self._vintercept = params.get('vintercept', DEFAULT_TURN_PARAMS['vintercept'])
        self._kv = params.get('kv', DEFAULT_TURN_PARAMS['kv'])
        self._ka = params.get('ka', DEFAULT_TURN_PARAMS['ka'])

    def compute(self, motor_pct: float, tm_diff: float) -> float:
        """Same TankModel physics but for angular motion"""
        appliedVoltage = self._nominalVoltage * motor_pct
        appliedVoltage = math.copysign(
            max(abs(appliedVoltage) - self._vintercept, 0), appliedVoltage
        )

        a0 = self.acceleration
        v0 = self.velocity
        v1 = v0 + a0 * tm_diff
        a1 = (appliedVoltage - self._kv * v1) / self._ka
        v1 = v0 + (a0 + a1) * 0.5 * tm_diff
        a1 = (appliedVoltage - self._kv * v1) / self._ka
        self.position += (v0 + v1) * 0.5 * tm_diff

        self.velocity = v1
        self.acceleration = a1
        return self.velocity

    def update(self, dt: float) -> float:
        """Update turning motor and return wrapped angle"""
        motor_power = self.motor.getAppliedOutput()
        self.compute(motor_power, dt)
        
        # Wrap angle for encoder
        wrapped_position = self.position % (2 * math.pi)
        self.sim_encoder.setVelocity(self.velocity)
        self.sim_encoder.setPosition(wrapped_position)
        
        return wrapped_position




class SwerveModel:
    def __init__(self, 
                 # 8 motors
                 fl_drive_motor, fl_turn_motor,
                 fr_drive_motor, fr_turn_motor,
                 bl_drive_motor, bl_turn_motor,
                 br_drive_motor, br_turn_motor,
                 
                 # DIFFERENT PARAMS PER MOTOR
                 fl_drive_kv, fl_drive_ka, fl_drive_vi,
                 fr_drive_kv, fr_drive_ka, fr_drive_vi,
                 bl_drive_kv, bl_drive_ka, bl_drive_vi,
                 br_drive_kv, br_drive_ka, br_drive_vi,
                 
                 fl_turn_kv, fl_turn_ka, fl_turn_vi,
                 fr_turn_kv, fr_turn_ka, fr_turn_vi,
                 bl_turn_kv, bl_turn_ka, bl_turn_vi,
                 br_turn_kv, br_turn_ka, br_turn_vi,
                 
                 # Physics
                 robot_mass, robot_width, robot_length, wheel_radius):
        
        # Create motors with individual params
        self.drive_motors = [
            SimpleTalonFXMotorSim(fl_drive_motor, 
                                 kv=fl_drive_kv, ka=fl_drive_ka, vintercept=fl_drive_vi),
            SimpleTalonFXMotorSim(fr_drive_motor,
                                 kv=fr_drive_kv, ka=fr_drive_ka, vintercept=fr_drive_vi),
            SimpleTalonFXMotorSim(bl_drive_motor,
                                 kv=bl_drive_kv, ka=bl_drive_ka, vintercept=bl_drive_vi),
            SimpleTalonFXMotorSim(br_drive_motor,
                                 kv=br_drive_kv, ka=br_drive_ka, vintercept=br_drive_vi)
        ]
        
        self.turn_motors = [
            SparkMaxTurningSim(fl_turn_motor,
                              kv=fl_turn_kv, ka=fl_turn_ka, vintercept=fl_turn_vi),
            SparkMaxTurningSim(fr_turn_motor,
                              kv=fr_turn_kv, ka=fr_turn_ka, vintercept=fr_turn_vi),
            SparkMaxTurningSim(bl_turn_motor,
                              kv=bl_turn_kv, ka=bl_turn_ka, vintercept=bl_turn_vi),
            SparkMaxTurningSim(br_turn_motor,
                              kv=br_turn_kv, ka=br_turn_ka, vintercept=br_turn_vi)
        ]
        
        # Module positions (geometry)
        half_w = robot_width / 2.0
        half_l = robot_length / 2.0
        self.module_positions = [
            (+half_l, +half_w),  # front-left
            (+half_l, -half_w),  # front-right
            (-half_l, +half_w),  # back-left
            (-half_l, -half_w),  # back-right
        ]
        
        # Physics parameters
        self.robot_mass = robot_mass
        self.robot_width = robot_width
        self.robot_length = robot_length
        self.wheel_radius = wheel_radius
        self.cof = 1.2
        self.g = 9.81
        
        # Derived constants
        self.robot_moi = (1 / 12) * robot_mass * (robot_width**2 + robot_length**2)
        self.normal_force_per_wheel = robot_mass * self.g / 4.0
        self.max_friction_force = self.cof * self.normal_force_per_wheel
        self.max_wheel_torque = self.max_friction_force * wheel_radius
        
        # Robot state 
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.vx = 0.0
        self.vy = 0.0

    def calculate(self, tm_diff: float) -> Transform2d:
        #takes tm_diff, returns robot motion
        
        # 1. Update all motors
        for drive_motor in self.drive_motors:
            drive_motor.update(tm_diff, self.wheel_radius, self.max_wheel_torque)
        for turn_motor in self.turn_motors:
            turn_motor.update(tm_diff)
        
        # 2. Calculate forces and torque from all modules
        total_fx = total_fy = total_torque = 0.0
        
        for i in range(4):
            # Get module angle and force
            angle = self.turn_motors[i].position
            wheel_force = self.drive_motors[i].last_torque / self.wheel_radius
            
            # Apply friction limits
            wheel_force = max(-self.max_friction_force, 
                             min(self.max_friction_force, wheel_force))
            
            # Force components
            fx = wheel_force * math.cos(angle)
            fy = wheel_force * math.sin(angle)
            total_fx += fx
            total_fy += fy
            
            # Torque contribution: τ = r × F
            rx, ry = self.module_positions[i]
            total_torque += rx * fy - ry * fx
        
        # 3. Apply physics: F=ma, torque=Ia
        ax = total_fx / self.robot_mass
        ay = total_fy / self.robot_mass
        angular_accel = total_torque / self.robot_moi
        
        # 4. Integrate velocities (like TankModel's integration)
        self.vx += ax * tm_diff
        self.vy += ay * tm_diff
        self.yaw_rate += angular_accel * tm_diff
        self.yaw += self.yaw_rate * tm_diff
        
        # 5. Calculate displacements
        dx = self.vx * tm_diff
        dy = self.vy * tm_diff
        dtheta = self.yaw_rate * tm_diff
        
        # 6. Convert to field-relative coordinates
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        field_dx = dx * cos_yaw - dy * sin_yaw
        field_dy = dx * sin_yaw + dy * cos_yaw
        
        return Transform2d(field_dx, field_dy, Rotation2d(dtheta))


#24 params is a lot, this is just in case that's too much
#TankModel.theory() calculates kv/ka from motor specs, so this is the equivalent to that

    @classmethod
    def theory(cls,
               # 8 motors
               fl_drive_motor, fl_turn_motor,
               fr_drive_motor, fr_turn_motor,
               bl_drive_motor, bl_turn_motor,
               br_drive_motor, br_turn_motor,
               
               # Robot physics
               robot_mass, robot_width, robot_length, wheel_radius,
               drive_gearing=6.75, turn_gearing=12.8,
               drive_motor_free_speed=6380,  # Falcon free speed in RPM
               drive_motor_stall_torque=4.69,  # Falcon stall torque in N·m
               turn_motor_free_speed=5880,     # NEO free speed in RPM
               turn_motor_stall_torque=2.6,    # NEO stall torque in N·m
               vintercept=1.3):
        
        
        
        # Calculate drive motor kv/ka (same formula as TankModel)
        max_linear_velocity = (drive_motor_free_speed * math.pi * wheel_radius * 2) / (60 * drive_gearing)
        max_linear_acceleration = (2 * drive_motor_stall_torque * drive_gearing) / (wheel_radius * robot_mass)
        
        drive_kv = 12.0 / max_linear_velocity  # V/(m/s)
        drive_ka = 12.0 / max_linear_acceleration  # V/(m/s²)
        
        # Calculate turn motor kv/ka (angular version)
        max_angular_velocity = (turn_motor_free_speed * 2 * math.pi) / (60 * turn_gearing)
        # Simplified angular inertia calculation
        max_angular_acceleration = (4 * turn_motor_stall_torque * turn_gearing) / (robot_mass * wheel_radius**2)
        
        turn_kv = 12.0 / max_angular_velocity  # V/(rad/s)
        turn_ka = 12.0 / max_angular_acceleration  # V/(rad/s²)
        
        # Create with calculated params (same for all motors initially)
        return cls(
            fl_drive_motor, fl_turn_motor,
            fr_drive_motor, fr_turn_motor,
            bl_drive_motor, bl_turn_motor,
            br_drive_motor, br_turn_motor,
            
            # Drive motors (all same)
            drive_kv, drive_ka, vintercept,  # FL
            drive_kv, drive_ka, vintercept,  # FR
            drive_kv, drive_ka, vintercept,  # BL
            drive_kv, drive_ka, vintercept,  # BR
            
            # Turn motors (all same)
            turn_kv, turn_ka, vintercept,  # FL
            turn_kv, turn_ka, vintercept,  # FR
            turn_kv, turn_ka, vintercept,  # BL
            turn_kv, turn_ka, vintercept,  # BR
            
            robot_mass, robot_width, robot_length, wheel_radius
        )
