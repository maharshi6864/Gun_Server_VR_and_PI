import cv2 as cv
from cv2 import aruco
import numpy as np
import socket
import threading
import json
import time

# === Load Calibration Parameters ===
camera_matrix = np.load("camera_matrix.npy")
dist_coeffs = np.load("dist_coeffs.npy")

marker_length = 0.04  # 5 cm

# === ArUco Marker Detection ===
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
param_markers = aruco.DetectorParameters()
param_markers.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
param_markers.adaptiveThreshWinSizeMin = 3
param_markers.adaptiveThreshWinSizeMax = 23
param_markers.adaptiveThreshWinSizeStep = 10
param_markers.minMarkerPerimeterRate = 0.03  # lower = detect smaller markers
param_markers.maxErroneousBitsInBorderRate = 0.04

# === UDP Settings ===
UDP_IP = "localhost"  # <-- SET YOUR VR CLIENT IP HERE
UDP_PORT = 9999
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# === Shared Variables ===
last_tvec = np.array([0.0, 0.0, 0.0], dtype=np.float32)
tvec_to_send = last_tvec.copy()

# === Start Video Capture ===
cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 1920) # set the width to 1920
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)
while True:
    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, _ = aruco.detectMarkers(gray, marker_dict, parameters=param_markers)

    marker_detected = False

    if marker_corners and marker_IDs is not None:
        ids = marker_IDs.flatten()
        for i, marker_id in enumerate(ids):
            if marker_id == 0:
                corners = marker_corners[i]

                _, tvec, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

                # Draw marker and axis
                cv.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 3)

                # Update tvec
                tvec_to_send = tvec[0].ravel()
                last_tvec = tvec_to_send.copy()
                marker_detected = True

                # Debug log
                # print(f"[POSE] x: {tvec_to_send[0]:.2f}, y: {tvec_to_send[1]:.2f}, z: {tvec_to_send[2]:.2f}")

    if not marker_detected:
        tvec_to_send = last_tvec.copy()

    # Prepare UDP payload
    payload = {
        "X": float(tvec_to_send[0]),
        "Y": float(tvec_to_send[1]),
        "Z": float(tvec_to_send[2]),
        "W": 0.0,
        "PITCH": 0.0,
        "YAW": 0.0,
        "ROLL": 0.0,
        "TRIGGER": False,
        "MAGAZINE": False,
        "BULLETS": 0,
        "MODE": 0,
        "MARKER_LOST": not marker_detected
    }

    udp_socket.sendto(json.dumps(payload).encode('utf-8'), (UDP_IP, UDP_PORT))

    # Display
    # cv.imshow("ArUco Tracking", frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
