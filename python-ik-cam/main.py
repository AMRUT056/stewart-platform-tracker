# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "opencv-python", 
#     "numpy", 
#     "pyserial", 
#     "pymodbus"
# ]
# ///

from concurrent.futures import ThreadPoolExecutor
import sys
import cv2
import numpy as np
import serial
import time
import re
import math
import os

from pymodbus.client import ModbusSerialClient

# --- CONFIGURATION ---
# Robot / OpenCM Port
ROBOT_PORT = 'COM6'      
# IMU Modbus Port (From the first script)
IMU_PORT = 'COM12'       
IMU_BAUDRATE = 9600
IMU_SLAVE_ID = 0x50
CAM_ID = 0

# Application Settings
TOOL_DELAY = 0.2
TAG_SIZE_CM = 9.0        # Main Tag Size
TARGET_ID = 0            # Main Tag ID
TAG_OFFSET_ID = 3        # Validation Tag ID
WAIT_TIME = 5
TAG3_EFFECTIVE_DIST_CM = 18.63 

# Logging
HEADER='INPUT COMMAND,OUTPUT_POS,OUTPUT_ORI'
os.makedirs("csv_outputs", exist_ok=1)
out_file = f"csv_outputs/imu_{time.time()}.csv"
with open(out_file, "a") as f:
    f.write('#the out orientation uses roll, pitch from imu and yaw from cam\n')
    f.write(HEADER + "\n")

# --- MODBUS / IMU HELPER FUNCTIONS ---
def to_signed_16(n):
    n = n & 0xFFFF
    return (n ^ 0x8000) - 0x8000

def normalize_angle(angle):
    """Keeps angle between -180 and 180."""
    while angle > 180: angle -= 360
    while angle < -180: angle += 360
    return angle

def get_imu_readings(client):
    """Reads registers from Modbus IMU and returns dictionary of values."""
    try:
        # Address 0x34 (52), count 12 registers
        rr = client.read_holding_registers(address=0x34, count=12, device_id=IMU_SLAVE_ID)
        if rr.isError():
            return None
        
        regs = rr.registers
        data = {}
        # We primarily need Roll, Pitch, Yaw (Registers 9, 10, 11)
        # Note: Registers index 9 corresponds to regs[9] in the list
        data['roll']  = to_signed_16(regs[9])  / 32768.0 * 180.0
        data['pitch'] = to_signed_16(regs[10]) / 32768.0 * 180.0
        data['yaw']   = to_signed_16(regs[11]) / 32768.0 * 180.0
        return data
    except Exception as e:
        # print(f"IMU Read Error: {e}")
        return None

# --- OPENCV / MATH HELPER FUNCTIONS ---
class PositionSmoother:
    def __init__(self, alpha=0.6):
        self.alpha = alpha
        self.prev_pos = None

    def update(self, pos):
        if self.prev_pos is None:
            self.prev_pos = np.array(pos)
            return pos
        curr_pos = np.array(pos)
        smooth_pos = self.alpha * curr_pos + (1 - self.alpha) * self.prev_pos
        self.prev_pos = smooth_pos
        return smooth_pos

def get_euler_angles(rvec):
    R, _ = cv2.Rodrigues(rvec)
    sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
    singular = sy < 1e-6
    
    if not singular:
        x_rot = math.atan2(R[2,1], R[2,2]) 
        y_rot = math.atan2(-R[2,0], sy)    
        z_rot = math.atan2(R[1,0], R[0,0]) 
    else:
        x_rot = math.atan2(-R[1,2], R[1,1])
        y_rot = math.atan2(-R[2,0], sy)
        z_rot = 0
    return np.degrees(x_rot), np.degrees(y_rot), np.degrees(z_rot)

def unwrap_angle(current_angle, previous_angle):
    if previous_angle is None: return current_angle
    diff = current_angle - previous_angle
    if diff > 180:
        current_angle -= 360
    elif diff < -180:
        current_angle += 360
    return current_angle

# --- GLOBAL VARIABLES ---
needed_data = [0,0]
last_command = ''
command_buffer = ""
typing_mode = False

# Tag 3 Offset Vector
TAG3_OFFSET_VEC = np.array([[TAG3_EFFECTIVE_DIST_CM], [0.0], [0.0]], dtype=np.float32)

# Camera Calibration
CALIB_WIDTH = 1280
CALIB_HEIGHT = 720
fx = 704.3734573939229904
fy = 703.6846317628715042
cx = 620.0124877195135014
cy = 344.2297177120070160
dist_coeffs = np.zeros((5, 1))

# 3D points for Tag 0
half_size = TAG_SIZE_CM / 2.0
obj_points = np.array([
    [-half_size, -half_size, 0], 
    [ half_size, -half_size, 0], 
    [ half_size,  half_size, 0], 
    [-half_size,  half_size, 0]  
], dtype=np.float32)

# --- INITIALIZATION ---
arduino = None
modbus_client = None

def init_robot():
    print(f"--- INITIALIZING ROBOT ({ROBOT_PORT}) ---")
    try:
        ard = serial.Serial(port=ROBOT_PORT, baudrate=115200, timeout=1)
        time.sleep(1) 
        
        print(f"Sending HOME command...")
        ard.write(b'0 0 0 0 0 0\n')
        
        print("Waiting 5 seconds for movement...")
        time.sleep(2)
        
        ard.reset_input_buffer()
        ard.reset_output_buffer()
        ard.timeout = 0 
        print("Robot Ready.")
        return ard
        
    except serial.SerialException as e:
        print(f"Error opening Robot serial port: {e}. Running in visual-only mode.")
        return None

def init_imu():
    print(f"--- INITIALIZING IMU ({IMU_PORT}) ---")
    client = ModbusSerialClient(port=IMU_PORT, baudrate=IMU_BAUDRATE, bytesize=8, parity='N', stopbits=1, timeout=0.1)
    if client.connect():
        print("IMU Connected.")
        return client
    else:
        print(f"Failed to connect to IMU at {IMU_PORT}")
        return None

arduino = init_robot()
modbus_client = init_imu()

show_me = ''
datas = [0,0,0,0,0,0]

def send_data(input_file, arduino_conn, wanandie=0):
    global last_command, show_me, datas
    datas_all = None
    time.sleep(WAIT_TIME * 2)
    try:
        with open(input_file, "r") as nf:
            datas_all = nf.readlines()
    except Exception as e:
        print(f"Error reading input file: {e}")
        return

    print("Queue:\n\t", datas_all)
    if arduino_conn:
        for i in datas_all:
            last_command = i.replace("\n", '')
            try:
                datas = [float(x) for x in last_command.split()]
            except:
                continue
                
            show_me = f"Moving to: {last_command}"
            print(f"sending {i}")
            arduino_conn.write(i.encode())
            time.sleep(WAIT_TIME)
    else:
        print("ERROR no arduino to send data to")            
    print("done")
    if wanandie: raise
            
if len(sys.argv) > 1:
    wannadie = 1 if len(sys.argv) > 2 else 0 
    print("Read Input : Successful")
    executor = ThreadPoolExecutor(max_workers=2)
    executor.submit(send_data, sys.argv[1], arduino, wannadie)

# --- CAMERA SETUP ---
cap = cv2.VideoCapture(CAM_ID) 
cap.set(cv2.CAP_PROP_FRAME_WIDTH, CALIB_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CALIB_HEIGHT)

ret, frame = cap.read()
if not ret:
    print("Error: Camera not responding.")
    exit()

h, w = frame.shape[:2]
print(f"Camera Resolution: {w}x{h}")

scale_x = w / CALIB_WIDTH
scale_y = h / CALIB_HEIGHT
current_camera_matrix = np.array([
    [fx * scale_x, 0.0,           cx * scale_x],
    [0.0,          fy * scale_y, cy * scale_y],
    [0.0,          0.0,          1.0]
], dtype=np.float32)

# --- TRACKING VARIABLES ---
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
parameters = cv2.aruco.DetectorParameters()
parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

smoother = PositionSmoother(alpha=0.15) 

current_imu_data = [0.0, 0.0, 0.0] # Roll, Pitch, Yaw

start_cam_angle_ref = None 
last_cam_angle = None

calib_done = False
calib_x = 0.0
calib_y = 0.0
calib_z = 0.0
calib_yaw_cam = 0.0
calib_roll_imu = 0.0
calib_pitch_imu = 0.0

print("\nTracking started. Press 'q' to quit. Press 'c' to recalibrate.")

while True:
    
    # --- 1. READ IMU (MODBUS) ---
    if modbus_client:
        imu_raw = get_imu_readings(modbus_client)
        if imu_raw:
            # We treat the values read here as "Absolute" relative to the sensor
            # The calibration logic below will handle taring.
            current_imu_data = [imu_raw['roll'], imu_raw['pitch'], imu_raw['yaw']]

            # Log to CSV if we are moving (have a last command)
            if last_command:
                # Basic debouncing/rate limiting could be added here if needed
                pass 
                
    # --- 2. LOGGING LOGIC (Preserved from original) ---
    # The original script logged inside the serial read block. 
    # Since we moved IMU reading out, we check here if we have fresh data and a command.
    if last_command and modbus_client:
         try:
             # We write one line per loop if command exists, essentially.
             # This might be verbose, but matches previous logic flow roughly.
            with open(out_file, "a") as f:
                f.write(f'"{last_command}","')
                if isinstance(needed_data[0], (list, tuple)):
                    for i in needed_data[0]:
                        f.write(f"{i:.2f},") 
                else:
                    f.write("0.0,0.0,0.0,") # Fallback
                
                # Use current IMU data
                f.write(f'","{current_imu_data[0]:.2f},{current_imu_data[1]:.2f},{needed_data[1]:.2f}"\n')
         except Exception as e:
             pass

    # --- 3. CAMERA PROCESS ---
    ret, frame = cap.read()
    if not ret: break
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)

    # Draw Optical Center
    scaled_cx = int(cx * scale_x)
    scaled_cy = int(cy * scale_y)
    cv2.drawMarker(frame, (scaled_cx, scaled_cy), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)

    raw_x, raw_y, raw_z = 0, 0, 0
    raw_cam_yaw = 0.0
    tag_detected = False
    
    rvec_0 = None
    tvec_0 = None

    if ids is not None:
        ids = ids.flatten()
        for i, tag_id in enumerate(ids):
            
            # Main Tracking Tag
            if tag_id == TARGET_ID:
                tag_detected = True
                current_corners = corners[i].reshape((4, 2))

                success, rvec, tvec = cv2.solvePnP(
                    obj_points, current_corners, 
                    current_camera_matrix, dist_coeffs, 
                    flags=cv2.SOLVEPNP_ITERATIVE
                )

                if success:
                    rvec_0 = rvec
                    tvec_0 = tvec
                    
                    raw_x = tvec[0][0]
                    raw_y = tvec[1][0]
                    raw_z = tvec[2][0]

                    _, _, raw_cam_z_rot = get_euler_angles(rvec)
                    
                    if start_cam_angle_ref is None:
                        start_cam_angle_ref = raw_cam_z_rot
                        last_cam_angle = raw_cam_z_rot
                    
                    continuous_angle = unwrap_angle(raw_cam_z_rot, last_cam_angle)
                    last_cam_angle = continuous_angle
                    raw_cam_yaw = continuous_angle

                    cv2.polylines(frame, [current_corners.astype(int)], True, (0, 255, 0), 2)
                    cv2.drawFrameAxes(frame, current_camera_matrix, dist_coeffs, rvec, tvec, TAG_SIZE_CM)

            # Validation Tag (Tag 3)
            elif tag_id == TAG_OFFSET_ID and rvec_0 is not None:
                    R_0, _ = cv2.Rodrigues(rvec_0)
                    T3_expected = tvec_0 + R_0 @ TAG3_OFFSET_VEC
                    points_3d = np.array([T3_expected.flatten()], dtype=np.float32)
                    T3_image_pt, _ = cv2.projectPoints(points_3d, rvec_0, tvec_0, current_camera_matrix, dist_coeffs)
                    exp_pt = T3_image_pt[0][0].astype(int)
                    cv2.circle(frame, tuple(exp_pt), 5, (255, 255, 255), -1)
                    cv2.polylines(frame, [corners[i].astype(int)], True, (255, 0, 0), 2)

    # --- CALIBRATION LOGIC ---
    if not calib_done and tag_detected:
        print("--- PERFORMING ZERO CALIBRATION ---")
        calib_x = raw_x
        calib_y = raw_y
        calib_z = raw_z
        calib_yaw_cam = raw_cam_yaw
        # Zero the IMU based on current readings
        calib_roll_imu = current_imu_data[0]
        calib_pitch_imu = current_imu_data[1]
        calib_done = True
        print(f"Offsets set. X:{calib_x:.2f}, Yaw:{calib_yaw_cam:.2f}, IMU_R:{calib_roll_imu:.2f}")

    # --- DISPLAY TABLE ---
    if calib_done and tag_detected:
        # Position Smoothing
        smooth_pos = smoother.update([raw_x, raw_y, raw_z])
        
        # Raw (Absolute) values
        abs_x, abs_y, abs_z = smooth_pos
        abs_yaw_cam = raw_cam_yaw
        abs_roll_imu = current_imu_data[0]
        abs_pitch_imu = current_imu_data[1]

        # Calibrated (Relative) values
        disp_x = abs_x - calib_x
        disp_y = abs_y - calib_y
        disp_z = abs_z - calib_z
        disp_yaw_cam = abs_yaw_cam - calib_yaw_cam
        
        # Apply normalization to IMU differences to handle wrapping if necessary
        # (Though roll/pitch usually don't wrap in this context, good practice)
        disp_roll_imu = normalize_angle(abs_roll_imu - calib_roll_imu)
        disp_pitch_imu = normalize_angle(abs_pitch_imu - calib_pitch_imu)
        
        needed_data[0] = (disp_x, disp_y, disp_z)
        needed_data[1] = disp_yaw_cam
        
        # --- DRAW TABLE ON SCREEN ---
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.6
        thick = 2
        color_lbl = (200, 200, 200) 
        color_raw = (0, 255, 255)   
        color_cal = (0, 255, 0)     
        
        # Table Layout
        x_label = 20
        x_input = 20+140
        x_raw = 160  + 140
        x_cal = 280 + 140
        y_start = 40 
        step = 25

        # Headers
        cv2.putText(frame, "PARAM", (x_label, y_start), font, scale, color_lbl, thick)
        cv2.putText(frame, "INPUT", (x_input, y_start), font, scale, (255, 255, 255), thick)
        cv2.putText(frame, "RAW",   (x_raw,   y_start), font, scale, color_raw, thick)
        cv2.putText(frame, "CALIB", (x_cal,   y_start), font, scale, color_cal, thick)
        cv2.line(frame, (x_label, y_start + 5), (x_cal + 60, y_start + 5), (255,255,255), 1)

        # Data Rows
        data_rows = [
            ("X",       datas[0], abs_x,       disp_x),
            ("Y",       datas[1], abs_y,       disp_y),
            ("Z",       datas[2], abs_z,       disp_z),
            ("Cam Yaw", datas[3], abs_yaw_cam, disp_yaw_cam),
            ("IMU Roll",datas[4], abs_roll_imu,disp_roll_imu),
            ("IMU Ptch",datas[5], abs_pitch_imu,disp_pitch_imu),
        ]

        for i, (label, input_val, raw_val, cal_val) in enumerate(data_rows):
            y = y_start + step + (i * step)
            cv2.putText(frame, label, (x_label, y), font, scale, color_lbl, thick)
            cv2.putText(frame, f"{input_val:.3f}", (x_input, y), font, scale, (255, 255, 255), thick)
            
            cv2.putText(frame, f"{raw_val:.2f}", (x_raw, y), font, scale, color_raw, thick)
            cv2.putText(frame, f"{cal_val:.2f}", (x_cal, y), font, scale, color_cal, thick)
        
        if show_me:
            cv2.rectangle(frame,(450,615),(840,655),(25,247,251), cv2.FILLED)
            cv2.putText(frame,show_me,(480,650),font, scale, (0 ,0,255), thick)
        
    elif not calib_done:
        cv2.putText(frame, "WAITING FOR TAG (Running Calibration Sequence)...", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        smoother.prev_pos = None

    # --- COMMAND INPUT OVERLAY ---
    if typing_mode:
        cv2.putText(frame, f"Cmd: {command_buffer}_", (20, h - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (222, 86, 160), 2)

    cv2.imshow("Stewart Platform Tracker", frame)
    key = cv2.waitKey(1) & 0xFF

    # --- KEYBOARD HANDLERS ---
    if key == ord('?') or key == ord('/'):
        typing_mode = True
        command_buffer = ""
        
    elif key == ord('b'):
        typing_mode=True
        command_buffer=last_command
        
    elif key == ord('c'): 
        # Recalibration logic
        print("\n--- RECALIBRATION TRIGGERED ---")
        calib_done = False
        start_cam_angle_ref = None 
        last_cam_angle = None
        
        # Send Home Command if arduino connected
        if arduino:
            arduino.write(b'0 0 0 0 0 0\n')
            print("Homing command sent...")
            # Note: We don't sleep 5s here to keep UI responsive, 
            # but usually you'd wait for physical homing.
        else:
            print("No Arduino connected for Homing.")

    elif typing_mode:
        if key == 13: # Enter
            if arduino:
                arduino.write((command_buffer + '\n').encode())
                print(f"Sent: {command_buffer}")
            typing_mode = False
            last_command = command_buffer
            try:
                datas = [float(x) for x in last_command.split()]
            except:
                pass
            command_buffer = ""
        elif key == 8: command_buffer = command_buffer[:-1]
        elif key == 27: typing_mode = False
        elif 32 <= key <= 126: command_buffer += chr(key)
    
    if key == ord('q'): break

cap.release()
if arduino: arduino.close()
if modbus_client: modbus_client.close()
cv2.destroyAllWindows()
