import cv2
import numpy as np
import mediapipe as mp
import serial
import time

# ---------------- Arduino Setup ----------------
arduino = None
try:
    arduino = serial.Serial('COM7', 115200, timeout=1)
    time.sleep(2)  # wait for Arduino to initialize
    print("✅ Arduino connected.")
except serial.SerialException:
    print("⚠️ Arduino not connected. Continuing without serial...")

# ---------------- MediaPipe Setup ----------------
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

# ---------------- Webcam Setup ----------------
cap = cv2.VideoCapture(1)
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
        print("Failed to grab frame.")
        break

    height, width, _ = frame.shape
    cross_x = width // 2
    cross_y = height // 2

    # Convert to RGB (MediaPipe uses RGB)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = pose.process(rgb_frame)

    delta_x = 0

    if results.pose_landmarks:
        landmarks = results.pose_landmarks.landmark

        # Use torso points for stability
        torso_indices = [
            mp_pose.PoseLandmark.LEFT_HIP,
            mp_pose.PoseLandmark.RIGHT_HIP,
            mp_pose.PoseLandmark.LEFT_SHOULDER,
            mp_pose.PoseLandmark.RIGHT_SHOULDER
        ]

        torso_points = []
        for idx in torso_indices:
            lm = landmarks[idx]
            if lm.visibility > 0.5:
                x_px = int(lm.x * width)
                y_px = int(lm.y * height)
                torso_points.append((x_px, y_px))
                cv2.circle(frame, (x_px, y_px), 5, (0, 255, 0), -1)

        if torso_points:
            torso_np = np.array(torso_points)
            com_x = int(np.mean(torso_np[:, 0]))
            com_y = int(np.mean(torso_np[:, 1]))
            cv2.circle(frame, (com_x, com_y), 8, (255, 0, 0), -1)

            delta_x = com_x - cross_x

    # ---------------- Send delta_x to Arduino ----------------
    if arduino is not None:
        try:
            arduino.write(f"{delta_x}\n".encode())
            print(f"Sent delta_x: {delta_x}")
        except Exception as e:
            print("Error sending to Arduino:", e)

    # ---------------- Display ----------------
    cv2.drawMarker(frame, (cross_x, cross_y), (0, 0, 255),
                   markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
    cv2.putText(frame, f"Delta X: {delta_x}", (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    cv2.imshow("MediaPipe Chair Tracker", frame)

    # Press ESC to exit
    if cv2.waitKey(1) & 0xFF == 27:
        break

# ---------------- Cleanup ----------------
cap.release()
cv2.destroyAllWindows()
if arduino is not None:
    arduino.close()
