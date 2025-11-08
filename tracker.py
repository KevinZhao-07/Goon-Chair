import cv2
import numpy as np
from ultralytics import YOLO
import mediapipe as mp

# ---------------- MediaPipe ----------------
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

# ---------------- YOLO ----------------
model = YOLO('yolov8n.pt')  # only used every n frames for correction

# ---------------- Webcam ----------------
cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("Cannot open webcam")
    exit()

cv2.namedWindow("Chair Tracker Hybrid", cv2.WINDOW_NORMAL)
cv2.moveWindow("Chair Tracker Hybrid", 100, 100)

delta_x, delta_y = 0, 0
frame_counter = 0
YOLO_INTERVAL = 15  # run YOLO every 15 frames

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    height, width, _ = frame.shape
    cross_x, cross_y = width // 2, height // 2

    # ---------------- MediaPipe Tracking ----------------
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = pose.process(rgb_frame)

    if results.pose_landmarks:
        landmarks = results.pose_landmarks.landmark
        # torso COM
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

            # MediaPipe updates delta_x every frame
            delta_x = com_x - cross_x
            delta_y = com_y - cross_y

            # optional: size for speed scaling
            x_min, y_min = np.min(torso_np, axis=0)
            x_max, y_max = np.max(torso_np, axis=0)
            box_area = (x_max - x_min) * (y_max - y_min)

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
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    # refine MediaPipe COM (optional smoothing)
                    com_x_yolo = (x1 + x2) // 2
                    com_y_yolo = (y1 + y2) // 2
                    # nudge MediaPipe COM slightly toward YOLO COM
                    com_x = int(0.7 * com_x + 0.3 * com_x_yolo)
                    com_y = int(0.7 * com_y + 0.3 * com_y_yolo)
                    # no direct delta_x overwrite from YOLO
                    break  # only first confident person

    # ---------------- Draw crosshair & delta_x ----------------
    cv2.drawMarker(frame, (cross_x, cross_y), (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
    cv2.putText(frame, f"Delta X (MediaPipe): {delta_x}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
    cv2.putText(frame, f"Delta Y: {delta_y}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

    cv2.imshow("Chair Tracker Hybrid", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
