#!/usr/bin/env python3
import os
os.environ['QT_QPA_PLATFORM'] = 'xcb'  # Force Qt to use X11 for OpenCV

import cv2
import numpy as np
import socket
import struct
import subprocess
import threading
import time
import json
import collections
import signal

# Graceful shutdown
running = True
def signal_handler(sig, frame):
    global running
    print("[Main] Shutdown signal received. Cleaning up...")
    running = False
    
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# === CONFIGURATION ===

WIFI_SSID     = "MIPL"
WIFI_PASSWORD = "a1b2c3d4"
IMU_UDP_PORT   = 8001
STATE_UDP_PORT = 8002
TCP_PORT       = 9999

MARKER_ID      = 0
MARKER_LENGTH  = 0.02  # meters
dx, dy, dz     = 0.000, 0.035, 0.535  # marker-to-pivot offset
marker_offset  = np.array([[dx], [dy], [dz]], dtype=np.float32)

# Camera calibration parameters
Fx, Fy = 769.89447688, 773.96654678
Cx, Cy = 373.61428207, 243.5124749
camera_matrix = np.array([[Fx,0,Cx],[0,Fy,Cy],[0,0,1]], dtype=np.float64)
dist_coeffs   = np.array([-0.71823127,0.71717924,-0.00966881,-0.001033975,-0.35173629], dtype=np.float64)

# Display settings
SHOW_CAMERA_FEED = True   # Set to False if you don't want to see the camera feed window
DISPLAY_WIDTH = 1280      # Default display width (will maintain aspect ratio)

# === MARKER VISIBILITY TRACKING ===
VISIBILITY_WINDOW = 1.0  # seconds
VISIBILITY_THRESHOLD = 0.15  # 15% threshold (adjust as needed)

# ArUco setup
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params = cv2.aruco.DetectorParameters()  # Use create method as suggested
params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
params.adaptiveThreshWinSizeMin = 3
params.adaptiveThreshWinSizeMax = 23
params.adaptiveThreshWinSizeStep = 10

# Shared state
data_lock = threading.Lock()
imu   = {'pitch':0.0,'roll':0.0,'yaw':0.0}
state = {'trigger':False,'magazine':False,'bullets':0,'mode':0}
pos   = {'x':0.0,'y':0.0,'z':0.0}
marker_visible = {'current':False, 'reliable':True}  # Marker visibility status

# Kalman filter setup with tuned noise covariances
fps = 60.0
dt = 1.0 / fps
kf = cv2.KalmanFilter(6,3)
kf.transitionMatrix = np.array([
    [1,0,0,dt,0,0],
    [0,1,0,0,dt,0],
    [0,0,1,0,0,dt],
    [0,0,0,1,0,0],
    [0,0,0,0,1,0],
    [0,0,0,0,0,1]
], dtype=np.float32)
kf.measurementMatrix   = np.hstack((np.eye(3), np.zeros((3,3)))).astype(np.float32)

# Tuned process and measurement noise covariances for smoother filtering
kf.processNoiseCov     = np.eye(6, dtype=np.float32) * 1e-3
kf.measurementNoiseCov = np.eye(3, dtype=np.float32) * 1e-1

kf.errorCovPost        = np.eye(6, dtype=np.float32)
kf.statePost           = np.zeros((6,1), dtype=np.float32)

# One Euro Filter with tuned parameters
class OneEuroFilter:
    def __init__(self, freq, min_cutoff=0.01, beta=1.0, dcutoff=1.0):
        self.freq = float(freq)
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.dcutoff = float(dcutoff)
        self.x_prev = None
        self.dx_prev = None
        self.last_time = None

    def _alpha(self, cutoff):
        tau = 1.0 / (2 * np.pi * cutoff)
        te = 1.0 / self.freq
        return 1.0 / (1.0 + tau / te)

    def filter(self, x):
        t = time.time()
        if self.last_time is None:
            dt = 1.0 / self.freq
        else:
            dt = t - self.last_time
        self.last_time = t
        if dt <= 0: dt = 1.0 / self.freq
        self.freq = 1.0 / dt

        x = x.flatten().astype(np.float32)
        if self.x_prev is None:
            self.x_prev = x
            self.dx_prev = np.zeros_like(x)
            return x.reshape(-1,1)

        dx = (x - self.x_prev) * self.freq
        a_d = self._alpha(self.dcutoff)
        dx_hat = a_d * dx + (1 - a_d) * self.dx_prev
        cutoff = self.min_cutoff + self.beta * np.abs(dx_hat)
        a = self._alpha(cutoff)
        x_hat = a * x + (1 - a) * self.x_prev

        self.x_prev = x_hat
        self.dx_prev = dx_hat
        return x_hat.reshape(-1,1)

euro_pos = OneEuroFilter(freq=fps, min_cutoff=0.1, beta=1.0)  # beta lowered to 1.0
euro_rot = OneEuroFilter(freq=fps, min_cutoff=0.1, beta=1.0)

# Create ArUcoDetector instance (recommended in recent OpenCV)
detector = cv2.aruco.ArucoDetector(aruco_dict, params)

# TCP server setup
tcp_conn = None
tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
tcp_socket.bind(("0.0.0.0", TCP_PORT))
tcp_socket.listen(1)

def tcp_listener():
    global tcp_conn, running
    tcp_socket.settimeout(1.0)  # timeout to check running flag
    while running:
        try:
            print("[TCP] Waiting for VR client...")
            conn, addr = tcp_socket.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"[TCP] Client connected from {addr}")
            tcp_conn = conn
        except socket.timeout:
            time.sleep(0.001)
            continue
        except Exception as e:
            print(f"[TCP] Error accepting connection: {e}")
            time.sleep(1)

def ensure_wifi():
    while True:
        try:
            socket.create_connection(("8.8.8.8", 53), timeout=3)
            print("[Wi-Fi] Connected")
            return
        except:
            print("[Wi-Fi] Reconnecting...")
            subprocess.call(["nmcli", "device", "wifi", "connect", WIFI_SSID, "password", WIFI_PASSWORD])
            time.sleep(1)

def imu_listener():
    global running
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", IMU_UDP_PORT))
    sock.setblocking(False)
    print(f"[IMU] Listening on UDP {IMU_UDP_PORT}")
    while running:
        try:
            data, _ = sock.recvfrom(12)
            p, r, y = struct.unpack('fff', data)
            with data_lock:
                imu['pitch'], imu['roll'], imu['yaw'] = p, r, y
        except BlockingIOError:
            time.sleep(0.001)  # small sleep to reduce CPU
        time.sleep(0.001)
    sock.close()

def state_listener():
    global running
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", STATE_UDP_PORT))
    sock.setblocking(False)
    print(f"[State] Listening on UDP {STATE_UDP_PORT}")
    while running:
        try:
            buf, _ = sock.recvfrom(3)
            if len(buf) == 3:
                f, b, m = struct.unpack('BBB', buf)
                with data_lock:
                    state['trigger']  = bool(f & 1)
                    state['magazine'] = bool(f & 2)
                    state['bullets']  = b
                    state['mode']     = m
        except BlockingIOError:
            time.sleep(0.001)
        time.sleep(0.001)
    sock.close()

def camera_thread():
    global running
    visibility_history = collections.deque(maxlen=int(VISIBILITY_WINDOW * fps))
    
    camera_id = 0
    max_retries = 5
    retry_delay = 2.0  # seconds
    
    half = MARKER_LENGTH / 2
    objp = np.array([
        [-half,  half, 0],
        [ half,  half, 0],
        [ half, -half, 0],
        [-half, -half, 0]
    ], dtype=np.float32)

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    kernel = np.ones((3,3),np.uint8)

    prev_world = np.zeros((3,1), dtype=np.float32)
    world_vel  = np.zeros((3,1), dtype=np.float32)
    alpha_vel  = 0.5
    
    if SHOW_CAMERA_FEED:
        cv2.namedWindow('AR Feed', cv2.WINDOW_NORMAL)
    
    while running:
        cap = None
        for attempt in range(max_retries):
            try:
                cap = cv2.VideoCapture(camera_id)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                cap.set(cv2.CAP_PROP_FPS, fps)
                
                if cap.isOpened():
                    native_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    native_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    print(f"[Camera] Successfully opened camera {camera_id} at {native_width}x{native_height}")
                    break
                else:
                    print(f"[Camera] Failed to open camera {camera_id}, attempt {attempt+1}/{max_retries}")
                    cap.release()
                    time.sleep(retry_delay)
            except Exception as e:
                print(f"[Camera] Error opening camera: {e}, attempt {attempt+1}/{max_retries}")
                if cap:
                    cap.release()
                time.sleep(retry_delay)
        
        if not cap or not cap.isOpened():
            print("[Camera] Could not open camera after multiple attempts. Waiting before retrying...")
            time.sleep(5)
            continue
            
        try:
            while running:
                ret, frame = cap.read()
                if not ret:
                    print("[Camera] Failed to get frame, reconnecting...")
                    break
                
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                gray = clahe.apply(gray)
                gray = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, kernel)

                pred = kf.predict()
                
                # Use new ArucoDetector API
                corners, ids, rejected = detector.detectMarkers(gray)
                
                marker_detected = ids is not None and MARKER_ID in ids.flatten()
                visibility_history.append(marker_detected)
                
                reliability_score = sum(visibility_history) / len(visibility_history) if visibility_history else 0
                is_reliable = reliability_score >= VISIBILITY_THRESHOLD
                
                with data_lock:
                    marker_visible['current'] = marker_detected
                    marker_visible['reliable'] = is_reliable
                
                if marker_detected:
                    idx = int(np.where(ids.flatten() == MARKER_ID)[0][0])

                    c = corners[idx].reshape(4,2).astype(np.float32)
                    
                    retval, rvec, tvec = cv2.solvePnP(objp, c, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
                    
                    if retval:
                        meas = np.array([[tvec[0]], [tvec[1]], [tvec[2]]], dtype=np.float32)
                        kf.correct(meas)
                        tvec = euro_pos.filter(meas).astype(np.float64)
                        rvec = euro_rot.filter(rvec.reshape(-1,1)).astype(np.float64)
                    else:
                        tvec = pred[0:3].astype(np.float64)
                        rvec = np.zeros((3,1), dtype=np.float64)
                    
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    try:
                        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
                    except Exception:
                        pass
                    
                    Rm, _ = cv2.Rodrigues(rvec)
                    raw_world = tvec + Rm.dot(marker_offset.astype(np.float64))
                    vel = (raw_world - prev_world) / dt
                    world_vel = alpha_vel * vel + (1 - alpha_vel) * world_vel
                    smooth_w = prev_world + world_vel * dt
                    prev_world = smooth_w
                    
                    final_world = euro_pos.filter(smooth_w.astype(np.float32))
                    
                    with data_lock:
                        pos['x'], pos['y'], pos['z'] = final_world.flatten().tolist()
                else:
                    tvec = pred[0:3].astype(np.float64)
                    rvec = np.zeros((3,1), dtype=np.float64)
                
                reliability_text = f"Marker {'reliable' if is_reliable else 'unreliable'} ({reliability_score*100:.1f}%)"
                cv2.putText(frame, reliability_text, (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, 
                            (0, 255, 0) if is_reliable else (0, 0, 255), 2)
                
                if SHOW_CAMERA_FEED:
                    # Resize window once outside loop would be better, but keeping here for safety
                    cv2.namedWindow('AR Feed', cv2.WINDOW_NORMAL)
                    height = int(frame.shape[0] * (DISPLAY_WIDTH / frame.shape[1]))
                    cv2.resizeWindow('AR Feed', DISPLAY_WIDTH, height)
                    cv2.imshow('AR Feed', frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    return
                
                time.sleep(0.001)  # small sleep to reduce CPU load
                    
        except Exception as e:
            print(f"[Camera] Error in camera processing: {e}")
        finally:
            if cap:
                cap.release()
            print("[Camera] Connection lost. Attempting to reconnect...")
            time.sleep(1)
            
    cv2.destroyAllWindows()

def send_loop():
    global tcp_conn, running
    while running:
        if tcp_conn is None:
            time.sleep(0.01)
            continue
        
        with data_lock:
            payload = json.dumps({
                'X': pos['x'], 'Y': pos['y'], 'Z': pos['z'],
                'PITCH': imu['pitch'], 'ROLL': imu['roll'], 'YAW': imu['yaw'],
                'TRIGGER': state['trigger'], 'MAGAZINE': state['magazine'],
                'BULLETS': state['bullets'], 'MODE': state['mode'],
                'MARKER_LOST': not marker_visible['reliable']  # Inverted to match name better
            })
            
        try:
            if tcp_conn:
                tcp_conn.sendall(payload.encode('utf-8'))
        except (BrokenPipeError, ConnectionResetError):
            print("[TCP] Connection lost.")
            tcp_conn = None
        
        time.sleep(0.005)

if __name__ == '__main__':
    try:
        ensure_wifi()
        threads = []
        threads.append(threading.Thread(target=imu_listener, daemon=True))
        threads.append(threading.Thread(target=state_listener, daemon=True))
        threads.append(threading.Thread(target=camera_thread, daemon=True))
        threads.append(threading.Thread(target=tcp_listener, daemon=True))
        
        for thread in threads:
            thread.start()
        
        send_loop()
    except KeyboardInterrupt:
        print("\n[Main] Interrupted by user. Shutting down...")
    finally:
        running = False
        print("[Main] Closing sockets and releasing resources...")
        if tcp_conn:
            tcp_conn.close()
        tcp_socket.close()
        print("[Main] Waiting for threads to finish...")
        for thread in threads:
            if thread.is_alive():
                thread.join(timeout=1.0)
        print("[Main] Cleanup complete. Exiting.")
        cv2.destroyAllWindows()
