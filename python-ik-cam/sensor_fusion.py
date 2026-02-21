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
import math
import os

from pymodbus.client import ModbusSerialClient

# --- CONFIGURATION ---
ROBOT_PORT = 'COM7'      
IMU_PORT = 'COM12'        
IMU_BAUDRATE = 9600
IMU_SLAVE_ID = 0x50
CAM_ID = 0

TAG_SIZE_CM = 9.0        
TARGET_ID = 0            
TAG_OFFSET_ID = 3        
TAG3_EFFECTIVE_DIST_CM = 18.63 

# Logging Setup
os.makedirs("csv_outputs", exist_ok=True)
HEADER = "TIMESTAMP,COMMAND,FUSED_X,FUSED_Y,FUSED_Z,FUSED_ROLL,FUSED_PITCH,FUSED_YAW"
out_file = f"csv_outputs/fused_live_{time.time():.0f}.csv"
with open(out_file, "a") as f:
    f.write(HEADER + "\n")

# --- HELPER FUNCTIONS ---
def to_signed_16(n):
    n = n & 0xFFFF
    return (n ^ 0x8000) - 0x8000

def normalize_angle(angle):
    while angle > 180: angle -= 360
    while angle < -180: angle += 360
    return angle

def get_imu_readings(client):
    try:
        rr = client.read_holding_registers(address=0x34, count=12, device_id=IMU_SLAVE_ID)
        if rr.isError(): return None
        regs = rr.registers
        return {
            'roll': to_signed_16(regs[9]) / 32768.0 * 180.0,
            'pitch': to_signed_16(regs[10]) / 32768.0 * 180.0,
            'yaw': to_signed_16(regs[11]) / 32768.0 * 180.0
        }
    except Exception:
        return None

def get_euler_angles(rvec):
    R, _ = cv2.Rodrigues(rvec)
    sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
    if sy >= 1e-6:
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
    if diff > 180: current_angle -= 360
    elif diff < -180: current_angle += 360
    return current_angle

# --- SENSOR FUSION CLASS ---
class RealTimeFuser:
    def __init__(self, yaw_alpha=0.96, pos_alpha=0.25):
        """
        yaw_alpha: 0.0 to 1.0. Higher = trust IMU more for yaw. Lower = trust camera more.
        pos_alpha: 0.0 to 1.0. Higher = faster position updates. Lower = smoother position.
        """
        self.yaw_alpha = yaw_alpha
        self.pos_alpha = pos_alpha
        self.prev_imu_yaw = None
        self.fused_yaw = None
        self.fused_pos = None

    def update(self, cam_pos, cam_yaw, imu_yaw):
        if self.fused_yaw is None or self.prev_imu_yaw is None:
            self.fused_yaw = cam_yaw
            self.prev_imu_yaw = imu_yaw
            self.fused_pos = np.array(cam_pos)
            return self.fused_pos, self.fused_yaw

        # 1. Yaw Fusion (Complementary)
        delta_imu_yaw = normalize_angle(imu_yaw - self.prev_imu_yaw)
        self.prev_imu_yaw = imu_yaw
        self.fused_yaw = self.yaw_alpha * (self.fused_yaw + delta_imu_yaw) + (1.0 - self.yaw_alpha) * cam_yaw

        # 2. Position Smoothing (EMA)
        self.fused_pos = self.pos_alpha * np.array(cam_pos) + (1.0 - self.pos_alpha) * self.fused_pos

        return self.fused_pos, self.fused_yaw

    def reset(self):
        self.prev_imu_yaw = None
        self.fused_yaw = None
        self.fused_pos = None

# --- GLOBAL VARIABLES ---
last_command = ''
command_buffer = ""
typing_mode = False
# Expected input order: X Y Z Roll Pitch Yaw
datas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 

TAG3_OFFSET_VEC = np.array([[TAG3_EFFECTIVE_DIST_CM], [0.0], [0.0]], dtype=np.float32)

CALIB_WIDTH = 1280
CALIB_HEIGHT = 720
fx = 704.3734573939229904
fy = 703.6846317628715042
cx = 620.0124877195135014
cy = 344.2297177120070160
dist_coeffs = np.zeros((5, 1))

half_size = TAG_SIZE_CM / 2.0
obj_points = np.array([
    [-half_size, -half_size, 0], 
    [ half_size, -half_size, 0], 
    [ half_size,  half_size, 0], 
    [-half_size,  half_size, 0]  
], dtype=np.float32)

# --- INITIALIZATION ---
def init_robot():
    print(f"--- INITIALIZING ROBOT ({ROBOT_PORT}) ---")
    try:
        ard = serial.Serial(port=ROBOT_PORT, baudrate=115200, timeout=1)
        time.sleep(1) 
        ard.write(b'0 0 0 0 0 0\n')
        time.sleep(2)
        ard.reset_input_buffer()
        ard.reset_output_buffer()
        ard.timeout = 0 
        print("Robot Ready.")
        return ard
    except serial.SerialException as e:
        print(f"Error opening Robot port: {e}. Visual-only mode.")
        return None

def init_imu():
    print(f"--- INITIALIZING IMU ({IMU_PORT}) ---")
    client = ModbusSerialClient(port=IMU_PORT, baudrate=IMU_BAUDRATE, bytesize=8, parity='N', stopbits=1, timeout=0.1)
    if client.connect():
        print("IMU Connected.")
        return client
    return None

arduino = init_robot()
modbus_client = init_imu()
fuser = RealTimeFuser(yaw_alpha=0.96, pos_alpha=0.25)
show_me = ''

def ang_diff(a, b):
    return 180 - abs(abs(a - b) - 180)

def wait_imu(settle_time=0.5, tolerance=0.5, timeout=15.0):
    if not modbus_client: return True
    time.sleep(0.2) 
    start_time = time.time()
    last_move_time = time.time()
    prev_readings = get_imu_readings(modbus_client)
    
    while time.time() - start_time < timeout:
        time.sleep(0.1)
        current_readings = get_imu_readings(modbus_client)
        if not current_readings or not prev_readings:
            prev_readings = current_readings
            continue
            
        d_roll = ang_diff(current_readings['roll'], prev_readings['roll'])
        d_pitch = ang_diff(current_readings['pitch'], prev_readings['pitch'])
        d_yaw = ang_diff(current_readings['yaw'], prev_readings['yaw'])
        
        if d_roll > tolerance or d_pitch > tolerance or d_yaw > tolerance:
            last_move_time = time.time()
        else:
            if time.time() - last_move_time >= settle_time:
                return True 
        prev_readings = current_readings 
    return False

def send_data(input_file, arduino_conn, wanandie=0):
    global last_command, show_me, datas
    try:
        with open(input_file, "r") as nf:
            datas_all = nf.readlines()
    except Exception as e:
        print(f"Error reading file: {e}")
        return

    if arduino_conn:
        for i in datas_all:
            last_command = i.replace("\n", '')
            try:
                # Expected format: X Y Z Roll Pitch Yaw
                datas = [float(x) for x in last_command.split()]
            except:
                continue
                
            show_me = f"Moving to: {last_command}"
            arduino_conn.write(i.encode())
            wait_imu()
    show_me=''
    if wanandie: raise
            
if len(sys.argv) > 1:
    wannadie = 1 if len(sys.argv) > 2 else 0 
    executor = ThreadPoolExecutor(max_workers=2)
    executor.submit(send_data, sys.argv[1], arduino, wannadie)

# --- CAMERA SETUP ---
cap = cv2.VideoCapture(CAM_ID) 
cap.set(cv2.CAP_PROP_FRAME_WIDTH, CALIB_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CALIB_HEIGHT)
ret, frame = cap.read()
if not ret: exit("Error: Camera not responding.")

h, w = frame.shape[:2]
scale_x, scale_y = w / CALIB_WIDTH, h / CALIB_HEIGHT
current_camera_matrix = np.array([
    [fx * scale_x, 0.0,          cx * scale_x],
    [0.0,          fy * scale_y, cy * scale_y],
    [0.0,          0.0,          1.0]
], dtype=np.float32)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
parameters = cv2.aruco.DetectorParameters()
parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# Tracking States
current_imu_data = [0.0, 0.0, 0.0] # Roll, Pitch, Yaw
last_cam_angle = None
calib_done = False
calib_fused_x, calib_fused_y, calib_fused_z = 0.0, 0.0, 0.0
calib_fused_yaw, calib_imu_roll, calib_imu_pitch = 0.0, 0.0, 0.0

print("\nTracking started. Press 'q' to quit. Press 'c' to recalibrate.")

while True:
    # 1. Read IMU
    if modbus_client:
        imu_raw = get_imu_readings(modbus_client)
        if imu_raw:
            current_imu_data = [imu_raw['roll'], imu_raw['pitch'], imu_raw['yaw']]

    # 2. Read Camera
    ret, frame = cap.read()
    if not ret: break
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)

    scaled_cx, scaled_cy = int(cx * scale_x), int(cy * scale_y)
    cv2.drawMarker(frame, (scaled_cx, scaled_cy), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)

    raw_x, raw_y, raw_z, raw_cam_yaw = 0.0, 0.0, 0.0, 0.0
    tag_detected = False
    
    if ids is not None:
        ids = ids.flatten()
        for i, tag_id in enumerate(ids):
            if tag_id == TARGET_ID:
                tag_detected = True
                current_corners = corners[i].reshape((4, 2))
                success, rvec, tvec = cv2.solvePnP(obj_points, current_corners, current_camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)

                if success:
                    raw_x, raw_y, raw_z = tvec[0][0], tvec[1][0], tvec[2][0]
                    _, _, raw_cam_z_rot = get_euler_angles(rvec)
                    
                    if last_cam_angle is None: last_cam_angle = raw_cam_z_rot
                    raw_cam_yaw = unwrap_angle(raw_cam_z_rot, last_cam_angle)
                    last_cam_angle = raw_cam_yaw

                    cv2.polylines(frame, [current_corners.astype(int)], True, (0, 255, 0), 2)
                    cv2.drawFrameAxes(frame, current_camera_matrix, dist_coeffs, rvec, tvec, TAG_SIZE_CM)

    # 3. Perform Sensor Fusion
    if tag_detected:
        fused_pos, fused_yaw_abs = fuser.update([raw_x, raw_y, raw_z], raw_cam_yaw, current_imu_data[2])
        abs_x, abs_y, abs_z = fused_pos
        abs_roll_imu = current_imu_data[0]
        abs_pitch_imu = current_imu_data[1]

        # 4. Handle Calibration
        if not calib_done:
            print("--- PERFORMING ZERO CALIBRATION ---")
            calib_fused_x, calib_fused_y, calib_fused_z = abs_x, abs_y, abs_z
            calib_fused_yaw = fused_yaw_abs
            calib_imu_roll = abs_roll_imu
            calib_imu_pitch = abs_pitch_imu
            calib_done = True

        # 5. Calculate Relative Displacements (Tared to zero)
        disp_x = abs_x - calib_fused_x
        disp_y = abs_y - calib_fused_y
        disp_z = abs_z - calib_fused_z
        disp_roll = normalize_angle(abs_roll_imu - calib_imu_roll)
        disp_pitch = normalize_angle(abs_pitch_imu - calib_imu_pitch)
        disp_yaw = fused_yaw_abs - calib_fused_yaw

        # 6. Logging
        if last_command:
            with open(out_file, "a") as f:
                f.write(f"{time.time():.3f},{last_command},{disp_x:.3f},{disp_y:.3f},{disp_z:.3f},{disp_roll:.3f},{disp_pitch:.3f},{disp_yaw:.3f}\n")

        # 7. UI Display (Aligned to X Y Z Roll Pitch Yaw format)
        font, scale, thick = cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
        color_lbl, color_val = (200, 200, 200), (0, 255, 0)
        
        y_start, step = 40, 25
        x_label, x_input, x_fused = 20, 160, 300
        
        cv2.putText(frame, "PARAM", (x_label, y_start), font, scale, color_lbl, thick)
        cv2.putText(frame, "COMMAND", (x_input, y_start), font, scale, (255, 255, 255), thick)
        cv2.putText(frame, "LIVE FUSED", (x_fused, y_start), font, scale, color_val, thick)
        cv2.line(frame, (x_label, y_start + 5), (x_fused + 100, y_start + 5), (255,255,255), 1)

        # Expected Input order: [X, Y, Z, Roll, Pitch, Yaw]
        data_rows = [
            ("X",     datas[0], disp_x),
            ("Y",     datas[1], disp_y),
            ("Z",     datas[2], disp_z),
            ("Roll",  datas[3], disp_roll),
            ("Pitch", datas[4], disp_pitch),
            ("Yaw",   datas[5], disp_yaw),
        ]

        for i, (label, input_val, fused_val) in enumerate(data_rows):
            y = y_start + step + (i * step)
            cv2.putText(frame, label, (x_label, y), font, scale, color_lbl, thick)
            cv2.putText(frame, f"{input_val:.3f}", (x_input, y), font, scale, (255, 255, 255), thick)
            cv2.putText(frame, f"{fused_val:.2f}", (x_fused, y), font, scale, color_val, thick)
        
        if show_me:
            cv2.rectangle(frame,(450,615),(840,655),(25,247,251), cv2.FILLED)
            cv2.putText(frame,show_me,(480,650),font, scale, (0,0,255), thick)

    elif not calib_done:
        cv2.putText(frame, "WAITING FOR TAG (Calibration)...", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # --- INPUT UI ---
    if typing_mode:
        cv2.putText(frame, f"Cmd: {command_buffer}_", (20, h - 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (222, 86, 160), 2)

    cv2.imshow("Real-Time Fused Tracking", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('?') or key == ord('/'):
        typing_mode = True
        command_buffer = ""
    elif key == ord('b'):
        typing_mode=True
        command_buffer=last_command
    elif key == ord('c'): 
        calib_done = False
        last_cam_angle = None
        fuser.reset()
        if arduino: arduino.write(b'0 0 0 0 0 0\n')
    elif typing_mode:
        if key == 13: 
            if arduino: arduino.write((command_buffer + '\n').encode())
            typing_mode = False
            last_command = command_buffer
            try: datas = [float(x) for x in last_command.split()]
            except: pass
            command_buffer = ""
        elif key == 8: command_buffer = command_buffer[:-1]
        elif key == 27: typing_mode = False
        elif 32 <= key <= 126: command_buffer += chr(key)
    
    if key == ord('q'): break

cap.release()
if arduino: arduino.close()
if modbus_client: modbus_client.close()
cv2.destroyAllWindows()