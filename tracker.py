import cv2
import numpy as np
from ultralytics import YOLO
import serial
import time

# # ---------------- Arduino Setup ----------------
# arduino = serial.Serial('COM7', 9600, timeout=1)
# time.sleep(2)

# ---------------- YOLO Setup ----------------
model = YOLO('yolov8n.pt')

# ---------------- Webcam Setup ----------------
cap = cv2.VideoCapture(1)
cv2.namedWindow("Chair Tracker YOLO", cv2.WINDOW_NORMAL)
cv2.moveWindow("Chair Tracker YOLO", 100, 100)

while True:
    ret, frame = cap.read()
    if not ret or frame is None:
        break

    height, width, _ = frame.shape

    results = model(frame)

    cross_x, cross_y = width // 2, height // 2
    cv2.drawMarker(frame, (cross_x, cross_y), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)

    delta_x = 0

    # Check for persons
    for result in results:
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            continue
        for box in boxes:
            cls = int(box.cls[0])
            if cls == 0:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                confidence = float(box.conf[0])
                if confidence < 0.5:
                    continue

                com_x = (x1 + x2) // 2
                delta_x = com_x - cross_x

                # Draw for visualization
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame, (com_x, (y1 + y2)//2), 6, (255, 0, 0), -1)
                break

    # Send deltaX to Arduino
    #arduino.write(f"{delta_x}\n".encode())

    # Display info
    cv2.putText(frame, f"Delta X: {delta_x}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255),2)
    cv2.imshow("Chair Tracker YOLO", frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
#arduino.close()
