# /// script
# requires-python = ">=3.13"
# dependencies = ["opencv-python", "numpy", "pyserial"]
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

HEADER='INPUT COMMAND,OUTPUT_POS,OUTPUT_ORI'
os.makedirs("csv_outputs",exist_ok=1)
out_file=f"csv_outputs/imu_{time.time()}.csv"
with open(out_file,"a") as f:
    f.write('#the out orientation uses roll, pitch from imu and yaw from cam\n')
    f.write(HEADER+"\n")

# --- HELPER CLASS FOR SMOOTHING ---
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

# --- HELPER FUNCTIONS FOR ROTATION ---
def get_euler_angles(rvec):
    """
    Converts Rodrigues vector to Euler angles.
    Returns: Pitch (X), Yaw (Y), Roll (Z - Normal Axis)
    """
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
    """
    Prevents the jump between 179 and -179 degrees.
    """
    if previous_angle is None: return current_angle
    diff = current_angle - previous_angle
    if diff > 180:
        current_angle -= 360
    elif diff < -180:
        current_angle += 360
    return current_angle


needed_data=[0,0]
last_command=''
command_buffer = ""
typing_mode = False

# --- 1. CONFIGURATION ---
PORT = 'COM4'           # Your COM port
TOOL_DELAY = 0.2
TAG_SIZE_CM = 9.0        # Main Tag Size
TARGET_ID = 0            # Main Tag ID
TAG_OFFSET_ID = 3        # Validation Tag ID
WAIT_TIME = 5
# [Requirement 1] Variable for Tag 3 Distance
TAG3_EFFECTIVE_DIST_CM = 18.63 

# Create the offset vector based on this variable (X-axis offset)
TAG3_OFFSET_VEC = np.array([[TAG3_EFFECTIVE_DIST_CM], [0.0], [0.0]], dtype=np.float32)

# --- 2. BASE CALIBRATION ---
CALIB_WIDTH = 1280
CALIB_HEIGHT = 720
fx = 704.3734573939229904
fy = 703.6846317628715042
cx = 620.0124877195135014
cy = 344.2297177120070160

dist_coeffs = np.zeros((5, 1))

# Define 3D points for Tag 0
half_size = TAG_SIZE_CM / 2.0
obj_points = np.array([
    [-half_size, -half_size, 0], 
    [ half_size, -half_size, 0], 
    [ half_size,  half_size, 0], 
    [-half_size,  half_size, 0]  
], dtype=np.float32)

# --- 3. SETUP SERIAL & STARTUP SEQUENCE ---
arduino = None
def init():
    print("--- INITIALIZING ROBOT ---")
    try:
        arduino = serial.Serial(port=PORT, baudrate=115200, timeout=1)
        time.sleep(1) 
        
        print(f"Sending HOME command to {PORT}...")
        arduino.write(b'0 0 0 0 0 0\n')
        
        print("Waiting 5 seconds for movement...")
        time.sleep(2)
        
        arduino.reset_input_buffer()
        arduino.reset_output_buffer()
        arduino.timeout = 0 
        print("Robot Ready.")
        return arduino
        
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}. Running in visual-only mode.")

arduino=init()
show_me=''
datas=[0,0,0,0,0,0]
def send_data(input,arduino,wanandie=0):
    global last_command,show_me,datas
    datas_all=None
    time.sleep(WAIT_TIME*2)
    with open(input,"r")as nf:
        datas_all=nf.readlines()
        
    print("Queue:\n\t",datas_all)
    if arduino:
        for i in datas_all:
            last_command=i.replace("\n",'')
            # datas=list(float(last_command.split(" ")))
            datas = [float(x) for x in last_command.split()]
            show_me=f"Moving to: {last_command}"
            print(f"sending {i}")
            arduino.write(i.encode())
            time.sleep(WAIT_TIME)
    else:
        print("ERROR no arduino")            
    print("done")
    if wanandie:raise
            
if len(sys.argv) > 1:
    wannadie = 1 if len(sys.argv)>2 else 0 
    print("Read Input : Successful")
    executor = ThreadPoolExecutor(max_workers=2)
    executor.submit(send_data,sys.argv[1],arduino,wannadie)

# --- 4. SETUP CAMERA ---
cap = cv2.VideoCapture(1) 
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
    [fx * scale_x, 0.0,          cx * scale_x],
    [0.0,          fy * scale_y, cy * scale_y],
    [0.0,          0.0,          1.0]
], dtype=np.float32)

# --- 5. VARIABLES INITIALIZATION ---
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
parameters = cv2.aruco.DetectorParameters()
parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

smoother = PositionSmoother(alpha=0.15) 

current_imu_data = [0.0, 0.0, 0.0] # Roll, Pitch, Yaw (Serial)

start_cam_angle_ref = None 
last_cam_angle = None

calib_done = False
calib_x = 0.0
calib_y = 0.0
calib_z = 0.0
calib_yaw_cam = 0.0
calib_roll_imu = 0.0
calib_pitch_imu = 0.0

print("\nTracking started. Press 'q' to quit. Press 'C' (Shift+c) to recalibrate.")

while True:
    # --- SERIAL READ ---
    if arduino:
        try:
            if arduino.in_waiting > 0:
                lines = arduino.read(arduino.in_waiting).decode(errors='ignore').split('\n')
                for line in reversed(lines):
                    if "[IMU]" in line:
                        match = re.search(r'Roll: ([-]?\d+\.\d+) \| Pitch: ([-]?\d+\.\d+) \| Yaw: ([-]?\d+\.\d+)', line)
                        if match:
                            current_imu_data = [
                                float(match.group(1)), 
                                float(match.group(2)), 
                                float(match.group(3))  
                            ]
                            show_me=''
                            with open(out_file,"a") as f:
                                if last_command: f.write(f'"{last_command}","')
                                for i in needed_data[0]:
                                    f.write(f"{i:.2f},") 
                                f.write(f'","{current_imu_data[0]},{current_imu_data[1]},{needed_data[1]:.2f}"\n')
                            break 
        except Exception as e:
            pass

    # --- CAMERA PROCESS ---
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
        calib_roll_imu = current_imu_data[0]
        calib_pitch_imu = current_imu_data[1]
        calib_done = True
        print(f"Offsets set. X:{calib_x:.2f}, Yaw:{calib_yaw_cam:.2f}, IMU_R:{calib_roll_imu:.2f}")

    # --- DISPLAY TABLE ---
    if calib_done and tag_detected:
        # Position Smoothing
        smooth_pos = smoother.update([raw_x, raw_y, raw_z])
        
        # Raw (Absolute) values (using smoothed position for X,Y,Z)
        abs_x, abs_y, abs_z = smooth_pos
        abs_yaw_cam = raw_cam_yaw
        abs_roll_imu = current_imu_data[0]
        abs_pitch_imu = current_imu_data[1]

        # Calibrated (Relative) values
        disp_x = abs_x - calib_x
        disp_y = abs_y - calib_y
        disp_z = abs_z - calib_z
        disp_yaw_cam = abs_yaw_cam - calib_yaw_cam
        disp_roll_imu = abs_roll_imu - calib_roll_imu
        disp_pitch_imu = abs_pitch_imu - calib_pitch_imu
        needed_data[0]=(disp_x,disp_y,disp_z)
        needed_data[1]=disp_yaw_cam
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

        for i, (label, input, raw_val, cal_val) in enumerate(data_rows):
            y = y_start + step + (i * step)
            cv2.putText(frame, label, (x_label, y), font, scale, color_lbl, thick)
            cv2.putText(frame, f"{input:.3f}", (x_input, y), font, scale, (255, 255, 255), thick)
            
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
        
    # [NEW REQUIREMENT] Shift+C for Recalibration
    elif key ==ord('b'):
        typing_mode=True
        command_buffer=last_command
    elif key == ord('c'): 
        if arduino:
            print("\n--- RECALIBRATION TRIGGERED (Shift+C) ---")
            
            # Reset Calibration Flag
            calib_done = False
            
            # Reset Angle Unwrapping State
            start_cam_angle_ref = None 
            last_cam_angle = None
            
            # Send Home Command
            arduino.write(b'0 0 0 0 0 0\n')
            print("Homing command sent. Waiting 5 seconds...")
            time.sleep(5)
            
            # Ensure buffers are clear before new readings
            # arduino.reset_input_buffer()
            # arduino.reset_output_buffer()
            print("Recalibration ready on next frame.")
        else:
            print("Serial port not open. Cannot recalibrate.")

    elif typing_mode:
        if key == 13: # Enter
            # print(arduino)
            if arduino:
                arduino.write((command_buffer + '\n').encode())
                print(f"Sent: {command_buffer}")
            typing_mode = False
            last_command=command_buffer
            datas = [float(x) for x in last_command.split()]
            command_buffer = ""
        elif key == 8: command_buffer = command_buffer[:-1]
        elif key == 27: typing_mode = False
        elif 32 <= key <= 126: command_buffer += chr(key)
    
    if key == ord('q'): break

cap.release()
if arduino: arduino.close()
cv2.destroyAllWindows()

