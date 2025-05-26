import cv2 as cv
from cv2 import aruco
import numpy as np

# Load your camera calibration parameters
# Replace these with your actual calibration data
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0,  0,  1]], dtype=np.float32)

dist_coeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float32)  # distortion coefficients

marker_length = 0.05  # Marker size in meters (set this correctly for your physical marker)

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
        # Filter for only ID 0
        ids = marker_IDs.flatten()
        for i, marker_id in enumerate(ids):
            if marker_id == 0:
                corners = marker_corners[i]

                # Draw marker border
                cv.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA)

                # Estimate pose of the marker
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corners, marker_length, camera_matrix, dist_coeffs
                )

                # Draw axis for the marker (length 0.03 m)
                aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec[0], tvec[0], 0.03)

                # Print or use pose info:
                print(f"ID 0 pose:")
                print(f"Translation vector (tvec): {tvec[0].ravel()}")  # x, y, z in meters
                print(f"Rotation vector (rvec): {rvec[0].ravel()}")     # Rodrigues rotation vector

                # You can put text on the frame too
                cv.putText(
                    frame,
                    f"ID: {marker_id}",
                    tuple(corners[0][0].astype(int)),
                    cv.FONT_HERSHEY_PLAIN,
                    1.3,
                    (200, 100, 0),
                    2,
                    cv.LINE_AA,
                )

    cv.imshow("frame", frame)
    key = cv.waitKey(1)
    if key == ord("q"):
        break

cap.release()
cv.destroyAllWindows()
