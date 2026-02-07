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
UR5_IP = "192.168.1.150"

# Mapping from UARM servo angles to UR5 joint positions
# You'll need to calibrate these ranges for your specific setup
UARM_MIN = np.array([0, 0, 0, 0, 0, 0, 0, 0])  # Min servo values
UARM_MAX = np.array([4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095])  # Max servo values

# UR5 joint limits (radians) - these are safe conservative limits
UR5_MIN = np.array([-2*np.pi, -2*np.pi, -2*np.pi, -2*np.pi, -2*np.pi, -2*np.pi])
UR5_MAX = np.array([2*np.pi, 2*np.pi, 2*np.pi, 2*np.pi, 2*np.pi, 2*np.pi])

# Control parameters
VELOCITY = 0.5  # rad/s - adjust for smoother/faster movement
ACCELERATION = 0.5  # rad/s^2
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
    
    for i in range(6):
        # Normalize UARM angle to 0-1
        normalized = (uarm_angles[i] - UARM_MIN[i]) / (UARM_MAX[i] - UARM_MIN[i])
        # Map to UR5 range
        ur5_joints[i] = UR5_MIN[i] + normalized * (UR5_MAX[i] - UR5_MIN[i])
    
    return ur5_joints

def clamp_joints(joints):
    """Clamp joint values to safe limits"""
    return np.clip(joints, UR5_MIN, UR5_MAX)

def check_safety(current_joints, target_joints, max_delta=0.1):
    """
    Safety check: ensure commanded motion isn't too large
    max_delta in radians per update
    """
    delta = np.abs(target_joints - current_joints)
    if np.any(delta > max_delta):
        print(f"⚠ Warning: Large motion detected! Delta: {delta}")
        return False
    return True

# ===== Main Control Loop =====
groupSyncRead = GroupSyncRead(packetHandler, SMS_STS_PRESENT_POSITION_L, 4)
angle_pos = np.zeros(7)

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
        
        # Safety check
        if check_safety(current_ur5_joints, target_ur5_joints):
            # Send command to UR5
            rtde_c.servoJ(target_ur5_joints.tolist(), VELOCITY, ACCELERATION, UPDATE_RATE, 0.2, 300)
        else:
            print("⚠ Safety check failed - skipping this command")
        
        # Display status
        print(f"UARM: {angle_pos[:6].astype(int)} -> UR5: {np.rad2deg(target_ur5_joints).astype(int)}")
        
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