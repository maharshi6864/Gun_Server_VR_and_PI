import cv2
from cv2 import aruco

marker_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
marker_id = 0
marker_size = 200  # pixels

marker_image = aruco.drawMarker(marker_dict, marker_id, marker_size)
cv2.imwrite(f"marker_{marker_id}.png", marker_image)
