# Teleoperation: UARM leader arm -> UR5 follower robot

import sys
import numpy as np
import time
from dataclasses import dataclass, field

sys.path.append("..")
from scservo_sdk import *

# UR5 control
import rtde_control
import rtde_receive


# ===== Configuration =====

@dataclass
class TeleopConfig:
    # UARM serial port
    # uarm_port: str = '/dev/cu.usbmodem5AE60548261'
    uarm_port: str = '/dev/TTYACM0'

    # UR5 network
    ur5_ip: str = "192.168.1.150"
    simulation_mode: bool = True

    # Servo parameters
    num_servos: int = 7
    servo_midpoint: int = 2047
    servo_range: int = 4095

    # UARM servo ranges (8 values, first 6 used for mapping)
    uarm_min: np.ndarray = field(default_factory=lambda: np.array([0, 0, 0, 0, 0, 0, 0, 0]))
    uarm_max: np.ndarray = field(default_factory=lambda: np.array([4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095]))

    # UR5 joint limits (radians)
    ur5_min: np.ndarray = field(default_factory=lambda: np.array([-np.pi, -np.pi, -np.pi, -np.pi, -np.pi, -3*np.pi/2]))
    ur5_max: np.ndarray = field(default_factory=lambda: np.array([np.pi, np.pi, np.pi, np.pi, np.pi, 3*np.pi/2]))

    # Inversion flags per joint
    invert: list = field(default_factory=lambda: [True, False, False, False, False, False])

    # Per-joint angular offsets (radians), applied after mapping
    joint_offsets: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, -7*np.pi/8, -3*np.pi/4, 0.0, 0.0]))

    # Control parameters
    velocity: float = 0.5
    acceleration: float = 0.5
    control_gain: float = 0.5
    update_rate: float = 0.01  # 100 Hz
    max_safe_delta: float = 0.25

    def __post_init__(self):
        if self.simulation_mode:
            self.ur5_ip = "localhost"


# ===== UARM Servo Reader =====

class UarmServoReader:
    def __init__(self, config: TeleopConfig):
        self.config = config
        self.port_handler = PortHandler(config.uarm_port)
        self.packet_handler = sms_sts(self.port_handler)
        self.group_sync_read = None

        if not self.port_handler.openPort():
            raise RuntimeError("Failed to open UARM port")
        print("Succeeded to open UARM port")

        if not self.port_handler.setBaudRate(1000000):
            raise RuntimeError("Failed to change the baudrate")
        print("Succeeded to change the baudrate")

        self.group_sync_read = GroupSyncRead(self.packet_handler, SMS_STS_PRESENT_POSITION_L, 4)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False

    def calibrate(self):
        """Run EPROM unlock/offset/lock calibration sequence for all servos."""
        print("\nCalibrating UARM servos...")
        for scs_id in range(1, self.config.num_servos + 1):
            self.packet_handler.unLockEprom(scs_id)
            time.sleep(0.1)

            comm, error = self.packet_handler.write2ByteTxRx(scs_id, SMS_STS_OFS_L, 0)
            time.sleep(0.1)

            raw_pos, result, error = self.packet_handler.read2ByteTxRx(scs_id, SMS_STS_PRESENT_POSITION_L)
            if result != COMM_SUCCESS:
                print(f"Failed to read position: {self.packet_handler.getTxRxResult(result)}")

            Homing_Offset = raw_pos - self.config.servo_midpoint
            if Homing_Offset < 0:
                encoded_offset = (1 << 11) | abs(Homing_Offset)
            else:
                encoded_offset = Homing_Offset

            comm, error = self.packet_handler.write2ByteTxRx(scs_id, SMS_STS_OFS_L, encoded_offset)
            if error == 0:
                print(f"Set half position for servo {scs_id}")
            time.sleep(0.1)

            self.packet_handler.LockEprom(scs_id)
            time.sleep(0.1)

    def read_positions(self) -> np.ndarray:
        """Read current positions from all servos via group sync read."""
        positions = np.zeros(self.config.num_servos)

        for scs_id in range(1, self.config.num_servos + 1):
            if not self.group_sync_read.addParam(scs_id):
                print(f"[ID:{scs_id:03d}] groupSyncRead addparam failed")

        scs_comm_result = self.group_sync_read.txRxPacket()
        if scs_comm_result != COMM_SUCCESS:
            print(f"Communication error: {self.packet_handler.getTxRxResult(scs_comm_result)}")
            self.group_sync_read.clearParam()
            return None

        for scs_id in range(1, self.config.num_servos + 1):
            scs_data_result, scs_error = self.group_sync_read.isAvailable(
                scs_id, SMS_STS_PRESENT_POSITION_L, 4
            )
            if scs_data_result:
                positions[scs_id - 1] = self.group_sync_read.getData(
                    scs_id, SMS_STS_PRESENT_POSITION_L, 2
                )
            else:
                print(f"[ID:{scs_id:03d}] groupSyncRead getdata failed")
            if scs_error != 0:
                print(f"Error: {self.packet_handler.getRxPacketError(scs_error)}")

        self.group_sync_read.clearParam()
        return positions

    def close(self):
        self.port_handler.closePort()


# ===== UR5 Controller =====

class UR5Controller:
    def __init__(self, config: TeleopConfig):
        self.config = config
        print("\nConnecting to UR5...")
        try:
            self.rtde_c = rtde_control.RTDEControlInterface(config.ur5_ip)
            self.rtde_r = rtde_receive.RTDEReceiveInterface(config.ur5_ip)
            print("Connected to UR5")
        except Exception as e:
            raise RuntimeError(f"Failed to connect to UR5: {e}")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type is KeyboardInterrupt:
            print("\n\nStopping teleoperation...")
            self.stop()
            print("UR5 stopped safely")
        self.close()
        return False

    def get_joint_positions(self) -> np.ndarray:
        return np.array(self.rtde_r.getActualQ())

    def move_to(self, joints, velocity, acceleration):
        self.rtde_c.moveJ(joints.tolist(), velocity, acceleration)

    def servo_to(self, joints, config: TeleopConfig):
        self.rtde_c.servoJ(
            joints.tolist(), config.velocity, config.acceleration,
            config.update_rate, 0.2, 300
        )

    def stop(self):
        self.rtde_c.servoStop()
        self.rtde_c.stopScript()

    def close(self):
        self.rtde_c.disconnect()
        self.rtde_r.disconnect()
        print("Disconnected from all devices")


# ===== Joint Mapper =====

class JointMapper:
    def __init__(self, config: TeleopConfig):
        self.config = config

    def map(self, uarm_positions: np.ndarray) -> np.ndarray:
        """Map UARM servo angles to UR5 joint positions."""
        ur5_joints = np.zeros(6)

        for i in range(6):
            # Normalize UARM angle to 0-1
            normalized = (uarm_positions[i] - self.config.uarm_min[i]) / (
                self.config.uarm_max[i] - self.config.uarm_min[i]
            )

            # Invert if needed
            if self.config.invert[i]:
                normalized = 1.0 - normalized

            # Map to UR5 range
            ur5_joints[i] = self.config.ur5_min[i] + normalized * (
                self.config.ur5_max[i] - self.config.ur5_min[i]
            )

        # Apply per-joint offsets
        ur5_joints += self.config.joint_offsets

        return ur5_joints

    def clamp(self, joints: np.ndarray) -> np.ndarray:
        """Clamp joint values to safe limits."""
        return np.clip(joints, self.config.ur5_min, self.config.ur5_max)

    def compute_safe_target(
        self, current: np.ndarray, target: np.ndarray, config: TeleopConfig
    ) -> np.ndarray:
        """Compute interpolated target with delta clamping for safety."""
        delta = np.abs(target - current)

        if np.any(delta > config.max_safe_delta):
            print("Large movement detected - clamping to safe speed")
            direction = target - current
            for i in range(6):
                if abs(direction[i]) > config.max_safe_delta:
                    direction[i] = np.sign(direction[i]) * config.max_safe_delta
            return current + config.control_gain * direction
        else:
            return current + config.control_gain * (target - current)


# ===== Main Logic =====

def initialize(uarm: UarmServoReader, ur5: UR5Controller, mapper: JointMapper, config: TeleopConfig):
    """Read initial UARM position, display info, and move UR5 to match."""
    print("\n" + "=" * 50)
    print("INITIALIZATION")
    print("=" * 50)
    print("The UR5 will move to match the leader arm position.")
    print("Ensure the area around the UR5 is clear!")
    input("\nPress ENTER when ready to initialize...")

    # Read initial UARM position
    positions = uarm.read_positions()
    if positions is None:
        raise RuntimeError("Failed to read initial UARM positions")

    # Calculate and display target position
    current_ur5 = ur5.get_joint_positions()
    target_position = mapper.clamp(mapper.map(positions))

    print(f"\nCurrent UR5: {np.rad2deg(current_ur5).astype(int)}")
    print(f"Target UR5:  {np.rad2deg(target_position).astype(int)}")
    print(f"UARM readings: {positions[:6].astype(int)}")
    print("\nMoving to initial position (this may take a few seconds)...")

    # Slowly move to target position
    ur5.move_to(target_position, 0.5, 0.3)

    print("Initialization complete!")
    time.sleep(0.5)


def control_loop(uarm: UarmServoReader, ur5: UR5Controller, mapper: JointMapper, config: TeleopConfig):
    """Run the teleoperation control loop until interrupted."""
    print("\n" + "=" * 50)
    print("Starting teleoperation control")
    print("Press Ctrl+C to stop")
    print("=" * 50 + "\n")

    while True:
        # Read UARM positions
        positions = uarm.read_positions()
        if positions is None:
            continue

        # Get current UR5 position
        current_ur5_joints = ur5.get_joint_positions()

        # Map UARM to UR5
        target_ur5_joints = mapper.clamp(mapper.map(positions))

        # Compute safe intermediate target
        intermediate_target = mapper.compute_safe_target(
            current_ur5_joints, target_ur5_joints, config
        )

        ur5.servo_to(intermediate_target, config)

        # Display status
        print(f"UARM: {positions[:6].astype(int)} -> UR5: {np.rad2deg(target_ur5_joints).astype(int)}")

        time.sleep(config.update_rate)


def run_teleoperation(config: TeleopConfig):
    """Run the full teleoperation sequence with proper resource cleanup."""
    with UarmServoReader(config) as uarm:
        uarm.calibrate()
        with UR5Controller(config) as ur5:
            mapper = JointMapper(config)
            initialize(uarm, ur5, mapper, config)
            control_loop(uarm, ur5, mapper, config)


if __name__ == "__main__":
    config = TeleopConfig()
    run_teleoperation(config)
