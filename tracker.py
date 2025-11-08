import cv2
import numpy as np
from ultralytics import YOLO
import mediapipe as mp
import serial
import time

# ---------------- Arduino Setup ----------------
arduino = None
try:
    arduino = serial.Serial('COM7', 9600, timeout=1)
    time.sleep(2)  # wait for Arduino to initialize
    print("Arduino connected.")
except serial.SerialException:
    print("Arduino not connected. Continuing without serial...")

# ---------------- MediaPipe ----------------
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

# ---------------- YOLO ----------------
model = YOLO('yolov8n.pt')  # YOLO for correction

# ---------------- Webcam ----------------
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

if not cap.isOpened():
    print("Cannot open webcam")
    exit()

cv2.namedWindow("Chair Tracker Hybrid", cv2.WINDOW_NORMAL)
cv2.moveWindow("Chair Tracker Hybrid", 100, 100)

# ---------------- Variables ----------------
delta_x = 0
frame_counter = 0
YOLO_INTERVAL = 15  # run YOLO every N frames

# ---------------- Main Loop ----------------
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    height, width, _ = frame.shape
    cross_x = width // 2

    # ---------------- MediaPipe Tracking ----------------
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = pose.process(rgb_frame)

    if results.pose_landmarks:
        landmarks = results.pose_landmarks.landmark
        torso_indices = [
            mp_pose.PoseLandmark.LEFT_HIP,
            mp_pose.PoseLandmark.RIGHT_HIP,
            mp_pose.PoseLandmark.LEFT_SHOULDER,
            mp_pose.PoseLandmark.RIGHT_SHOULDER
        ]

        torso_points_x = []
        for idx in torso_indices:
            lm = landmarks[idx]
            if lm.visibility > 0.5:
                x_px = int(lm.x * width)
                torso_points_x.append(x_px)
                cv2.circle(frame, (x_px, height // 2), 5, (0, 255, 0), -1)

        if torso_points_x:
            com_x = int(np.mean(torso_points_x))
            cv2.circle(frame, (com_x, height // 2), 8, (255, 0, 0), -1)
            delta_x = com_x - cross_x

    # ---------------- YOLO Correction ----------------
    frame_counter += 1
    if frame_counter % YOLO_INTERVAL == 0:
        results_yolo = model(frame)
        for result in results_yolo:
            boxes = result.boxes
            if boxes is None or len(boxes) == 0:
                continue
            for box in boxes:
                cls = int(box.cls[0])
                if cls == 0 and float(box.conf[0]) > 0.5:  # person
                    x1, x2 = int(box.xyxy[0][0]), int(box.xyxy[0][2])
                    com_x_yolo = (x1 + x2) // 2
                    delta_x = int(0.7 * delta_x + 0.3 * (com_x_yolo - cross_x))
                    break  # only first confident person

    # ---------------- Send delta_x to Arduino ----------------
    if arduino is not None:
        try:
            arduino.write(f"{delta_x}\n".encode())
            print(f"Sent delta_x to Arduino: {delta_x}")
        except Exception as e:
            print("Error sending to Arduino:", e)

    # ---------------- Display ----------------
    cv2.drawMarker(frame, (cross_x, height // 2), (0, 0, 255),
                   markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
    cv2.putText(frame, f"Delta X: {delta_x}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    cv2.imshow("Chair Tracker Hybrid", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
if arduino is not None:
    arduino.close()
