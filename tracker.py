import cv2
import numpy as np
import mediapipe as mp
import serial
import time
from threading import Thread
from queue import Queue

# ---------------- Arduino Setup ----------------
arduino_port = 'COM7'
arduino_baud = 115200
arduino = None

try:
    arduino = serial.Serial(arduino_port, arduino_baud, timeout=1)
    time.sleep(2)
    print("✅ Arduino connected.")
except serial.SerialException:
    print("⚠️ Arduino not connected. Continuing without serial...")

# ---------------- Queue for delta_x ----------------
delta_queue = Queue()

def serial_writer():
    """Thread to write delta_x to Arduino without blocking OpenCV."""
    global arduino
    while True:
        if not delta_queue.empty() and arduino is not None:
            dx = delta_queue.get()
            try:
                arduino.write(f"{dx}\n".encode())
                # Optional: read Arduino response if you want
            except serial.SerialException as e:
                print("⚠️ Arduino disconnected during write:", e)
                arduino.close()
                arduino = None

Thread(target=serial_writer, daemon=True).start()

# ---------------- MediaPipe Setup ----------------
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

# ---------------- Webcam Setup ----------------
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print("❌ Cannot open webcam")
    exit()

cv2.namedWindow("MediaPipe Chair Tracker", cv2.WINDOW_NORMAL)
cv2.moveWindow("MediaPipe Chair Tracker", 100, 100)

# ---------------- Main Loop ----------------
while True:
    ret, frame = cap.read()
    if not ret:
        continue

    height, width, _ = frame.shape
    cross_x = width // 2
    cross_y = height // 2

    # Convert to RGB for MediaPipe
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    try:
        results = pose.process(rgb_frame)
    except Exception as e:
        print("MediaPipe error:", e)
        continue

    delta_x = 0
    torso_points = []

    if results.pose_landmarks:
        landmarks = results.pose_landmarks.landmark
        torso_indices = [
            mp_pose.PoseLandmark.LEFT_HIP,
            mp_pose.PoseLandmark.RIGHT_HIP,
            mp_pose.PoseLandmark.LEFT_SHOULDER,
            mp_pose.PoseLandmark.RIGHT_SHOULDER
        ]

        for idx in torso_indices:
            lm = landmarks[idx]
            if lm.visibility > 0.5:
                x_px = int(lm.x * width)
                y_px = int(lm.y * height)
                torso_points.append((x_px, y_px))

        if torso_points:
            torso_np = np.array(torso_points)
            com_x = int(np.mean(torso_np[:, 0]))
            com_y = int(np.mean(torso_np[:, 1]))
            delta_x = com_x - cross_x

    # ---------------- Send delta_x to Arduino ----------------
    if arduino is not None:
        delta_queue.put(delta_x)

    # ---------------- Display ----------------
    display_frame = cv2.flip(frame, 1)
    cv2.drawMarker(display_frame, (cross_x, cross_y), (0, 0, 255),
                   markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
    cv2.putText(display_frame, f"Delta X: {delta_x}", (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    if results.pose_landmarks:
        for idx in torso_indices:
            lm = results.pose_landmarks.landmark[idx]
            if lm.visibility > 0.5:
                x_px = int(lm.x * width)
                y_px = int(lm.y * height)
                cv2.circle(display_frame, (width - x_px, y_px), 5, (0, 255, 0), -1)
        if torso_points:
            cv2.circle(display_frame, (width - com_x, com_y), 8, (255, 0, 0), -1)

    cv2.imshow("MediaPipe Chair Tracker", display_frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

# ---------------- Cleanup ----------------
cap.release()
cv2.destroyAllWindows()
if arduino is not None:
    arduino.close()
