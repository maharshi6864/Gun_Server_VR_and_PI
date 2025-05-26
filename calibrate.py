import cv2

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920) # set the width to 1920
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
img_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break
    cv2.imshow("Calibration Capture", frame)
    key = cv2.waitKey(1)

    if key == 32:  # Space bar to save image
        img_name = f"calib_image_{img_count}.png"
        cv2.imwrite(img_name, frame)
        print(f"Saved {img_name}")
        img_count += 1
    elif key == 27:  # ESC to quit
        break

cap.release()
cv2.destroyAllWindows()
