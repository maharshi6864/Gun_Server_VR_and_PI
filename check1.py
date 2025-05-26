import cv2 as cv
from cv2 import aruco
import numpy as np

# Load camera calibration parameters (from your calibration step)
camera_matrix = np.load("camera_matrix.npy")
dist_coeffs = np.load("dist_coeffs.npy")

marker_length = 0.05  # Set this to your actual marker size in meters

# ArUco dictionary and detector parameters
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
param_markers = aruco.DetectorParameters()

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

                # Print pose info (optional, prints in console)
                print(f"Marker ID 0 translation vector (x, y, z): {tvec[0].ravel()}")
                print(f"Marker ID 0 rotation vector (rvec): {rvec[0].ravel()}")

    cv.imshow("Aruco Detection with Pose", frame)
    key = cv.waitKey(1)
    if key == ord("q"):
        break

cap.release()
cv.destroyAllWindows()
