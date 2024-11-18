import cv2
import numpy as np

windowName = "Ball Detection"


cameraIndex = 0 


cap = cv2.VideoCapture(cameraIndex)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_height, frame_width = frame.shape[:2]
    center_x = frame_width // 2
    center_y = frame_height // 2

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)

    circles = cv2.HoughCircles(
        blurred, 
        cv2.HOUGH_GRADIENT, 
        dp=1.2,  # Inverse ratio of the accumulator resolution to image resolution
        minDist = 50,  # Minimum distance between detected centers
        param1 = 102,  # Higher threshold for Canny
        param2 = 33,  # Accumulator threshold for circle detection
        minRadius = 30,  # Minimum radius of circles
        maxRadius = 60  # Maximum radius of circles
    )
    
    if circles is not None:
        circles = np.uint16(np.around(circles)).astype(np.int32)

        i = circles[0, 0]
        cv2.circle(frame, (i[0], i[1]), i[2], (0, 255, 0), -1, 8, 0)
        cv2.circle(frame, (i[0], i[1]), 2, (0, 0, 255), 3, 8, 0)
        
        relative_x = i[0] - center_x 
        relative_y = center_y - i[1]
        
        print(f"Circle at (x, y) = ({relative_x}, {relative_y}) with radius {i[2]}")

    cv2.circle(frame, (center_x, center_y), 2, (255, 0, 0), -1)
    cv2.imshow(windowName, frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()