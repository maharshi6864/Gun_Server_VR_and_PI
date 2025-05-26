import cv2 as cv
from cv2 import aruco
import numpy as np
import socket
import threading

# Load camera calibration parameters (from your calibration step)
camera_matrix = np.load("camera_matrix.npy")
dist_coeffs = np.load("dist_coeffs.npy")

marker_length = 0.05  # Set this to your actual marker size in meters

# ArUco dictionary and detector parameters
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
param_markers = aruco.DetectorParameters()

# TCP server parameters
TCP_IP = "0.0.0.0"
TCP_PORT = 9999

# Shared variable for translation vector, thread safe with a lock
tvec_to_send = None
tvec_lock = threading.Lock()

# List of connected clients
clients = []
clients_lock = threading.Lock()

def client_handler(conn, addr):
    print(f"[TCP] Client connected from {addr}")
    try:
        while True:
            # Keep connection alive - just wait, no data expected from client
            data = conn.recv(1024)
            if not data:
                break
    except Exception as e:
        print(f"[TCP] Client {addr} error: {e}")
    finally:
        print(f"[TCP] Client disconnected: {addr}")
        with clients_lock:
            clients.remove(conn)
        conn.close()

def tcp_server():
    global clients
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((TCP_IP, TCP_PORT))
    server.listen()
    print(f"[TCP] Server listening on {TCP_IP}:{TCP_PORT}")
    while True:
        conn, addr = server.accept()
        with clients_lock:
            clients.append(conn)
        threading.Thread(target=client_handler, args=(conn, addr), daemon=True).start()

# Start TCP server thread
threading.Thread(target=tcp_server, daemon=True).start()

cap = cv.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, rejected = aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers
    )

    if marker_corners and marker_IDs is not None:
        ids = marker_IDs.flatten()
        for i, marker_id in enumerate(ids):
            if marker_id == 0:
                corners = marker_corners[i]

                # Draw detected marker boundary
                cv.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA)

                # Estimate pose of marker ID 0
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corners, marker_length, camera_matrix, dist_coeffs
                )

                # Draw the axis on the marker (length 0.03 meters)
                cv.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec[0], tvec[0], 0.03)

                # Show ID text
                cv.putText(
                    frame,
                    f"ID: {marker_id}",
                    tuple(corners[0][0].astype(int)),
                    cv.FONT_HERSHEY_PLAIN,
                    1.3,
                    (0, 100, 255),
                    2,
                    cv.LINE_AA,
                )

                # Save translation vector thread-safely to send over TCP
                with tvec_lock:
                    tvec_to_send = tvec[0].ravel()

                # Print pose info (optional, prints in console)
                print(f"Marker ID 0 translation vector (x, y, z): {tvec_to_send}")
                print(f"Marker ID 0 rotation vector (rvec): {rvec[0].ravel()}")

    # Send translation vector to all connected clients (non-blocking)
    if tvec_to_send is not None:
        data_str = f"{tvec_to_send[0]:.5f},{tvec_to_send[1]:.5f},{tvec_to_send[2]:.5f}\n"
        data_bytes = data_str.encode('utf-8')
        with clients_lock:
            disconnected = []
            for c in clients:
                try:
                    c.sendall(data_bytes)
                except Exception:
                    # Mark client for removal on error (e.g. disconnected)
                    disconnected.append(c)
            for dc in disconnected:
                clients.remove(dc)
                dc.close()

    cv.imshow("Aruco Detection with Pose", frame)
    key = cv.waitKey(1)
    if key == ord("q"):
        break

cap.release()
cv.destroyAllWindows()
