# /// script
# requires-python = ">=3.13"
# dependencies = ["opencv-python", "numpy", "pyserial", "pymodbus"]
# ///

import cv2
import numpy as np
import serial
import time
import math
from pymodbus.client import ModbusSerialClient

# --- CONFIGURATION (Match your main script) ---
ROBOT_PORT = 'COM7'      
IMU_PORT = 'COM12'        
CAM_ID = 0
TAG_SIZE_CM = 9.0        

# Tolerances for diagnostics
NOISE_THRESHOLD = 1.0     # Ignore movements smaller than this (mm or degrees)
ERROR_MARGIN = 0.3        # 30% error allowed for scaling issues
CROSSTALK_MARGIN = 0.5    # If an unintended axis moves >50% of the intended one

# --- SENSOR HELPERS ---
def to_signed_16(n): return ((n & 0xFFFF) ^ 0x8000) - 0x8000
def normalize_angle(angle): return (angle + 180) % 360 - 180

def get_imu_readings(client):
    try:
        rr = client.read_holding_registers(address=0x34, count=12, device_id=0x50)
        if rr.isError(): return None
        regs = rr.registers
        return [to_signed_16(regs[9])/32768.0*180.0, to_signed_16(regs[10])/32768.0*180.0, to_signed_16(regs[11])/32768.0*180.0]
    except: return None

# Camera setup essentials
cap = cv2.VideoCapture(CAM_ID)
fx, fy, cx, cy = 704.37, 703.68, 620.01, 344.23 
camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((5, 1))
obj_points = np.array([[-TAG_SIZE_CM/2, -TAG_SIZE_CM/2, 0], [TAG_SIZE_CM/2, -TAG_SIZE_CM/2, 0], 
                       [TAG_SIZE_CM/2, TAG_SIZE_CM/2, 0], [-TAG_SIZE_CM/2, TAG_SIZE_CM/2, 0]], dtype=np.float32)
detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5))

def get_camera_readings():
    # Flushes buffer and grabs fresh frame
    for _ in range(5): cap.read() 
    ret, frame = cap.read()
    if not ret: return None
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is not None and 0 in ids:
        idx = np.where(ids == 0)[0][0]
        success, rvec, tvec = cv2.solvePnP(obj_points, corners[idx], camera_matrix, dist_coeffs)
        if success:
            R, _ = cv2.Rodrigues(rvec)
            yaw = np.degrees(math.atan2(R[1,0], R[0,0]))
            return [tvec[0][0], tvec[1][0], tvec[2][0], yaw]
    return None

# --- DIAGNOSTIC ENGINE ---
def analyze_movement(cmd, start_cam, start_imu, end_cam, end_imu):
    print(f"\n[{'='*40}]")
    print(f"ANALYZING COMMAND: {cmd}")
    
    # Calculate Deltas
    dx = end_cam[0] - start_cam[0]
    dy = end_cam[1] - start_cam[1]
    dz = end_cam[2] - start_cam[2]
    dcam_yaw = normalize_angle(end_cam[3] - start_cam[3])
    
    droll = normalize_angle(end_imu[0] - start_imu[0])
    dpitch = normalize_angle(end_imu[1] - start_imu[1])
    dimu_yaw = normalize_angle(end_imu[2] - start_imu[2])

    actuals = [dx, dy, dz, droll, dpitch, dimu_yaw] # Use IMU for roll/pitch, IMU for yaw in this test
    axis_names = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
    
    # Find intended axis
    intended_idx = -1
    for i, val in enumerate(cmd):
        if abs(val) > 0:
            intended_idx = i
            break
            
    if intended_idx == -1:
        print("Command was HOMING (0,0,0,0,0,0). Skipping diagnosis.")
        return

    expected_val = cmd[intended_idx]
    actual_val = actuals[intended_idx]
    intended_axis = axis_names[intended_idx]

    print(f"Intended Movement: {expected_val} on {intended_axis}")
    print(f"Measured Deltas: X:{dx:.1f} Y:{dy:.1f} Z:{dz:.1f} R:{droll:.1f} P:{dpitch:.1f} Yw:{dimu_yaw:.1f}")

    # --- RULE 1: Did it move at all? ---
    if all(abs(a) < NOISE_THRESHOLD for a in actuals):
        print("❌ ERROR: NO MOVEMENT DETECTED.")
        print("   Diagnosis: Hardware failure, bad serial connection, or input parsing failed on the Arduino.")
        return

    # --- RULE 2: Did the WRONG axis move? (Cross-talk / Misalignment) ---
    max_actual_idx = np.argmax(np.abs(actuals))
    if max_actual_idx != intended_idx and abs(actuals[max_actual_idx]) > abs(actual_val):
        wrong_axis = axis_names[max_actual_idx]
        print(f"❌ ERROR: SEVERE CROSS-TALK. Commanded {intended_axis}, but {wrong_axis} moved the most.")
        
        if intended_idx in [0, 1] and max_actual_idx in [0, 1]:
            print("   Diagnosis: Camera is rotated 90 degrees relative to the robot's X/Y axes.")
        elif intended_idx in [3, 4] and max_actual_idx in [3, 4]:
            print("   Diagnosis: IMU is mounted sideways (rotated 90 degrees on the Z axis).")
        else:
            print("   Diagnosis: Severe Kinematic misconfiguration on the Arduino. Axes are mapped to the wrong motors.")
        return

    # --- RULE 3: Is it moving BACKWARDS? (Inversion) ---
    if (expected_val > 0 and actual_val < -NOISE_THRESHOLD) or (expected_val < 0 and actual_val > NOISE_THRESHOLD):
        print(f"❌ ERROR: INVERTED AXIS on {intended_axis}.")
        if intended_idx in [0, 1, 2]:
            print("   Diagnosis: Camera is mounted upside down/backwards, OR Arduino kinematics have reversed motor directions.")
        else:
            print("   Diagnosis: IMU is mounted upside down, OR orientation parsing is inverted in code.")
        return

    # --- RULE 4: Scale factor issues ---
    error_ratio = abs(abs(actual_val) - abs(expected_val)) / abs(expected_val)
    if error_ratio > ERROR_MARGIN:
        print(f"⚠️ WARNING: SCALE MISMATCH. Expected {expected_val}, got {actual_val:.1f}.")
        if intended_idx in [0, 1, 2]:
            print("   Diagnosis: TAG_SIZE_CM is wrong, camera focal length (fx/fy) is uncalibrated, OR hardware is not physically capable of reaching the commanded distance.")
        else:
            print("   Diagnosis: Platform geometry (base radius, platform radius in Arduino code) is incorrect, limiting rotational reach.")

    # --- If we pass the checks ---
    if error_ratio <= ERROR_MARGIN and (expected_val * actual_val > 0):
        print(f"✅ SUCCESS: {intended_axis} axis looks correctly calibrated and oriented.")

# --- MAIN EXECUTION ---
print("Connecting...")
ard = serial.Serial(ROBOT_PORT, 115200, timeout=1)
client = ModbusSerialClient(port=IMU_PORT, baudrate=9600, timeout=0.1)
client.connect()
time.sleep(2)

with open("calibration_path.txt", "r") as f:
    commands = [[float(x) for x in line.split()] for line in f if line.strip()]

for cmd in commands:
    # 1. Get baseline
    start_cam = get_camera_readings()
    start_imu = get_imu_readings(client)
    
    if not start_cam or not start_imu:
        print("Sensor error. Make sure Tag is visible and IMU is connected.")
        break
        
    # 2. Send command
    cmd_str = " ".join(map(str, cmd)) + "\n"
    ard.write(cmd_str.encode())
    print(f"\nSent: {cmd_str.strip()} ... waiting for physical movement.")
    
    # 3. Wait for settling (Hardcoded for calibration reliability)
    time.sleep(3.5) 
    
    # 4. Get final state
    end_cam = get_camera_readings()
    end_imu = get_imu_readings(client)
    
    # 5. Diagnose
    analyze_movement(cmd, start_cam, start_imu, end_cam, end_imu)

print("\n--- CALIBRATION DIAGNOSTICS COMPLETE ---")
ard.close()
client.close()
cap.release()