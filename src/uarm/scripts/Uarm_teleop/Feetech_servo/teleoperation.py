# Teleoperation: UARM leader arm -> UR5 follower robot

import sys
import numpy as np
import time

sys.path.append("..")
from scservo_sdk import *

# UR5 control
import rtde_control
import rtde_receive

# ===== Configuration =====
UARM_PORT = '/dev/ttyACM0'

SIMULATION_MODE = True
if SIMULATION_MODE:
    UR5_IP = "localhost"  # CB3 URSim
else:
    UR5_IP = "192.168.1.150"  # Your real CB3 robot

# Mapping from UARM servo angles to UR5 joint positions
# You'll need to calibrate these ranges for your specific setup
UARM_MIN = np.array([0, 0, 0, 0, 0, 0, 0, 0])  # Min servo values
UARM_MAX = np.array([4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095])  # Max servo values

# UR5 joint limits (radians) - these are safe conservative limits
UR5_MIN = np.array([-2*np.pi, -2*np.pi, -2*np.pi, -2*np.pi, -2*np.pi, -2*np.pi])
UR5_MAX = np.array([2*np.pi, 2*np.pi, 2*np.pi, 2*np.pi, 2*np.pi, 2*np.pi])

# Control parameters
VELOCITY = 0.05  # rad/s - adjust for smoother/faster movement
ACCELERATION = 0.05  # rad/s^2
CONTROL_GAIN = 0.2
UPDATE_RATE = 0.01  # 100 Hz update rate

# ===== Initialize UARM =====
portHandler = PortHandler(UARM_PORT)
packetHandler = sms_sts(portHandler)

if portHandler.openPort():
    print("✓ Succeeded to open UARM port")
else:
    print("✗ Failed to open UARM port")
    quit()

if portHandler.setBaudRate(1000000):
    print("✓ Succeeded to change the baudrate")
else:
    print("✗ Failed to change the baudrate")
    quit()

# Set all servos' half position (your original calibration code)
print("\nCalibrating UARM servos...")
for scs_id in range(1, 8):
    packetHandler.unLockEprom(scs_id)
    time.sleep(0.1)
    
    comm, error = packetHandler.write2ByteTxRx(scs_id, SMS_STS_OFS_L, 0)
    time.sleep(0.1)
    
    raw_pos, result, error = packetHandler.read2ByteTxRx(scs_id, SMS_STS_PRESENT_POSITION_L)
    if result != COMM_SUCCESS:
        print(f"Failed to read position: {packetHandler.getTxRxResult(result)}")
    
    Homing_Offset = raw_pos - 2047
    if Homing_Offset < 0:
        encoded_offset = (1 << 11) | abs(Homing_Offset)
    else:
        encoded_offset = Homing_Offset
    
    comm, error = packetHandler.write2ByteTxRx(scs_id, SMS_STS_OFS_L, encoded_offset)
    if error == 0:
        print(f"✓ Set half position for servo {scs_id}")
    time.sleep(0.1)
    
    packetHandler.LockEprom(scs_id)
    time.sleep(0.1)

# ===== Initialize UR5 =====
print("\nConnecting to UR5...")
try:
    rtde_c = rtde_control.RTDEControlInterface(UR5_IP)
    rtde_r = rtde_receive.RTDEReceiveInterface(UR5_IP)
    print("✓ Connected to UR5")
except Exception as e:
    print(f"✗ Failed to connect to UR5: {e}")
    quit()

# ===== Helper Functions =====
def map_uarm_to_ur5(uarm_angles):
    """
    Map UARM servo angles to UR5 joint positions
    Modify this function based on your kinematic mapping
    """
    # Simple linear mapping - you'll need to calibrate this!
    # This assumes first 6 UARM servos map to 6 UR5 joints
    ur5_joints = np.zeros(6)

    # Inversion flags: True = invert this joint, False = normal
    # 1 TRUE
    # 2 FALSE
    # 3 TRUE
    # 4 TRUE?
    # 5 TRUE?
    # 6
    invert = [True, False, True, True, True, False]  # Adjust per joint as needed
    
    for i in range(6):
        # Normalize UARM angle to 0-1
        normalized = (uarm_angles[i] - UARM_MIN[i]) / (UARM_MAX[i] - UARM_MIN[i])

        # Invert if needed
        if invert[i]:
            normalized = 1.0 - normalized

        # Map to UR5 range
        ur5_joints[i] = UR5_MIN[i] + normalized * (UR5_MAX[i] - UR5_MIN[i])
    
    return ur5_joints

def clamp_joints(joints):
    """Clamp joint values to safe limits"""
    return np.clip(joints, UR5_MIN, UR5_MAX)

def check_safety(current_joints, target_joints, max_delta=0.25):
    """
    Safety check: ensure commanded motion isn't too large
    max_delta in radians per update
    """
    delta = np.abs(target_joints - current_joints)
    if np.any(delta > max_delta):
        print(f"⚠ Warning: Large motion detected! Delta: {delta}")
        return False
    return True

# ===== Setup Group Read =====
groupSyncRead = GroupSyncRead(packetHandler, SMS_STS_PRESENT_POSITION_L, 4)
angle_pos = np.zeros(7)

# ===== Gradual Initialization =====
print("\n" + "="*50)
print("INITIALIZATION")
print("="*50)
print("The UR5 will move to match the leader arm position.")
print("Ensure the area around the UR5 is clear!")
input("\nPress ENTER when ready to initialize...")

# Read initial UARM position
for scs_id in range(1, 8):
    scs_addparam_result = groupSyncRead.addParam(scs_id)
    if not scs_addparam_result:
        print(f"[ID:{scs_id:03d}] groupSyncRead addparam failed")

scs_comm_result = groupSyncRead.txRxPacket()
if scs_comm_result != COMM_SUCCESS:
    print(f"Communication error: {packetHandler.getTxRxResult(scs_comm_result)}")

for scs_id in range(1, 8):
    scs_data_result, scs_error = groupSyncRead.isAvailable(scs_id, SMS_STS_PRESENT_POSITION_L, 4)
    if scs_data_result:
        angle_pos[scs_id - 1] = groupSyncRead.getData(scs_id, SMS_STS_PRESENT_POSITION_L, 2)

groupSyncRead.clearParam()

# Calculate and display target position
current_ur5 = np.array(rtde_r.getActualQ())
target_position = map_uarm_to_ur5(angle_pos)
target_position = clamp_joints(target_position)

print(f"\nCurrent UR5: {np.rad2deg(current_ur5).astype(int)}°")
print(f"Target UR5:  {np.rad2deg(target_position).astype(int)}°")
print(f"UARM readings: {angle_pos[:6].astype(int)}")
print("\nMoving to initial position (this may take a few seconds)...")

# Slowly move to target position
rtde_c.moveJ(target_position.tolist(), 0.5, 0.3)

print("✓ Initialization complete!")
time.sleep(0.5)

# ===== Main Control Loop =====
print("\n" + "="*50)
print("Starting teleoperation control")
print("Press Ctrl+C to stop")
print("="*50 + "\n")

try:
    while True:
        # Read UARM positions
        for scs_id in range(1, 8):
            scs_addparam_result = groupSyncRead.addParam(scs_id)
            if scs_addparam_result != True:
                print(f"[ID:{scs_id:03d}] groupSyncRead addparam failed")
        
        scs_comm_result = groupSyncRead.txRxPacket()
        if scs_comm_result != COMM_SUCCESS:
            print(f"Communication error: {packetHandler.getTxRxResult(scs_comm_result)}")
            groupSyncRead.clearParam()
            continue
        
        for scs_id in range(1, 8):
            scs_data_result, scs_error = groupSyncRead.isAvailable(scs_id, SMS_STS_PRESENT_POSITION_L, 4)
            if scs_data_result == True:
                scs_present_position = groupSyncRead.getData(scs_id, SMS_STS_PRESENT_POSITION_L, 2)
                angle_pos[scs_id - 1] = scs_present_position
            else:
                print(f"[ID:{scs_id:03d}] groupSyncRead getdata failed")
                continue
            if scs_error != 0:
                print(f"Error: {packetHandler.getRxPacketError(scs_error)}")
        
        groupSyncRead.clearParam()
        
        # Get current UR5 position
        current_ur5_joints = np.array(rtde_r.getActualQ())
        
        # Map UARM to UR5
        target_ur5_joints = map_uarm_to_ur5(angle_pos)
        target_ur5_joints = clamp_joints(target_ur5_joints)
        
        delta = np.abs(target_ur5_joints - current_ur5_joints)
        max_safe_delta = 0.25

        if np.any(delta > max_safe_delta):
            # Movement too large - clamp it to max safe distance
            print(f"⚠ Large movement detected - clamping to safe speed")
            direction = target_ur5_joints - current_ur5_joints
            # Normalize and scale to max safe delta
            for i in range(6):
                if abs(direction[i]) > max_safe_delta:
                    direction[i] = np.sign(direction[i]) * max_safe_delta
            intermediate_target = current_ur5_joints + CONTROL_GAIN * direction
        else:
            # Normal operation
            intermediate_target = current_ur5_joints + CONTROL_GAIN * (target_ur5_joints - current_ur5_joints)

        # ADD THIS LINE:
        rtde_c.servoJ(intermediate_target.tolist(), VELOCITY, ACCELERATION, UPDATE_RATE, 0.2, 300)

        # Display status
        print(f"UARM: {angle_pos[:6].astype(int)} -> UR5: {np.rad2deg(target_ur5_joints).astype(int)}°")
        
        time.sleep(UPDATE_RATE)

except KeyboardInterrupt:
    print("\n\nStopping teleoperation...")
    rtde_c.servoStop()
    rtde_c.stopScript()
    print("✓ UR5 stopped safely")

finally:
    # Cleanup
    portHandler.closePort()
    rtde_c.disconnect()
    rtde_r.disconnect()
    print("✓ Disconnected from all devices")