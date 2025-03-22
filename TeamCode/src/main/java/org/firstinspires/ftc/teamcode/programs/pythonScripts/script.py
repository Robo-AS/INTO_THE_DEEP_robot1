import cv2
import numpy as np

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()


lower_bound = np.array([100, 150, 70])
upper_bound = np.array([125, 255, 255])

color_detected = False

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break

    detected = False

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) > 500:
            detected = True

            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0


            cv2.circle(frame, (cX, cY), 7, (0, 255, 0), -1)
            cv2.putText(frame, "Target color detected!", (cX - 20, cY - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)


    if detected and not color_detected:
        print("DETECTED")
        color_detected = True
    elif not detected and color_detected:
        print("NOT DETECTED")
        color_detected = False123


    cv2.imshow('Webcam', frame)
    cv2.imshow('Mask', mask)


    if cv2.waitKey(1) & 0xFF == ord('m'):
        break

cap.release()
cv2.destroyAllWindows()