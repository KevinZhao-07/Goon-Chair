import cv2
import numpy as np
import mediapipe as mp
import serial
import time
from threading import Thread
from queue import Queue
import asyncio
import websockets

# ---------------- Arduino Setup ----------------
arduino_port = 'COM5'
arduino_baud = 115200
arduino = None
try:
    arduino = serial.Serial(arduino_port, arduino_baud, timeout=1)
    time.sleep(2)
    print("✅ Arduino connected.")
except serial.SerialException:
    print("⚠️ Arduino not connected. Continuing without serial...")

# ---------------- Serial Writer Thread ----------------
command_queue = Queue()  # commands to Arduino

def serial_writer():
    global arduino
    while True:
        if not command_queue.empty() and arduino:
            cmd = command_queue.get()
            try:
                arduino.write(f"{cmd}\n".encode())
            except serial.SerialException:
                print("⚠️ Serial write failed. Closing port.")
                arduino.close()
                arduino = None

Thread(target=serial_writer, daemon=True).start()

# ---------------- MediaPipe Setup ----------------
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

# ---------------- Webcam Setup ----------------
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print("❌ Cannot open webcam")
    if arduino:
        command_queue.put(9999)
    exit()

cv2.namedWindow("MediaPipe Chair Tracker", cv2.WINDOW_NORMAL)
cv2.moveWindow("MediaPipe Chair Tracker", 100, 100)

# ---------------- Global Command ----------------
current_command = "stop"  # default
COMMAND_STOP = 9999       # Arduino stop sentinel

# ---------------- WebSocket Handler ----------------
async def ws_handler(websocket):
    global current_command
    async for message in websocket:
        if message in ["stop", "track", "scan"]:
            print(f"✅ Received command from UI: {message}")
            current_command = message
        else:
            print("❌ Unknown command:", message)

# ---------------- Start WebSocket Server in Thread ----------------
def start_ws_thread():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    server = websockets.serve(ws_handler, "127.0.0.1", 8765)
    loop.run_until_complete(server)
    print("🌐 WebSocket server started on port 8765")
    loop.run_forever()

Thread(target=start_ws_thread, daemon=True).start()

# ---------------- Main Loop ----------------
sweep_direction = 1
sweep_speed = 50

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    height, width, _ = frame.shape
    cross_x = width // 2
    delta_x = COMMAND_STOP
    torso_points = []

    if current_command == "track":
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        try:
            results = pose.process(rgb_frame)
        except Exception as e:
            print("MediaPipe error:", e)
            delta_x = COMMAND_STOP
        else:
            if results.pose_landmarks:
                landmarks = results.pose_landmarks.landmark
                torso_indices = [mp_pose.PoseLandmark.LEFT_HIP,
                                 mp_pose.PoseLandmark.RIGHT_HIP,
                                 mp_pose.PoseLandmark.LEFT_SHOULDER,
                                 mp_pose.PoseLandmark.RIGHT_SHOULDER]
                for idx in torso_indices:
                    lm = landmarks[idx]
                    if lm.visibility > 0.5:
                        x_px = int(lm.x * width)
                        y_px = int(lm.y * height)
                        torso_points.append((x_px, y_px))
                if torso_points:
                    com_x = int(np.mean([p[0] for p in torso_points]))
                    delta_x = com_x - cross_x
                else:
                    delta_x = COMMAND_STOP
            else:
                delta_x = COMMAND_STOP

    elif current_command == "scan":
        delta_x = sweep_speed * sweep_direction
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = pose.process(rgb_frame)
        torso_points = []
        if results.pose_landmarks:
            torso_indices = [mp_pose.PoseLandmark.LEFT_HIP,
                             mp_pose.PoseLandmark.RIGHT_HIP,
                             mp_pose.PoseLandmark.LEFT_SHOULDER,
                             mp_pose.PoseLandmark.RIGHT_SHOULDER]
            for idx in torso_indices:
                lm = results.pose_landmarks.landmark[idx]
                if lm.visibility > 0.5:
                    x_px = int(lm.x * width)
                    y_px = int(lm.y * height)
                    torso_points.append((x_px, y_px))
            if torso_points:
                delta_x = COMMAND_STOP

        if delta_x > width // 2:
            sweep_direction = -1
        elif delta_x < -width // 2:
            sweep_direction = 1

    if arduino is not None:
        while not command_queue.empty():
            command_queue.get_nowait()
        command_queue.put(delta_x)

    # ---------------- Display ----------------
    display_frame = frame.copy()
    cv2.drawMarker(display_frame, (cross_x, height//2), (0,0,255),
                   markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
    for (x, y) in torso_points:
        cv2.circle(display_frame, (x, y), 5, (0,255,0), -1)
    if torso_points:
        com_x = int(np.mean([p[0] for p in torso_points]))
        cv2.circle(display_frame, (com_x, height//2), 8, (255,0,0), -1)

    text = "Stopped" if delta_x == COMMAND_STOP else f"deltaX = {delta_x}"
    color = (0,0,255) if delta_x == COMMAND_STOP else (255,255,255)
    cv2.putText(display_frame, text, (10,40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

    cv2.imshow("MediaPipe Chair Tracker", display_frame)
    if cv2.waitKey(1) & 0xFF == 27:
        if arduino:
            command_queue.put(COMMAND_STOP)
        break

cap.release()
cv2.destroyAllWindows()
if arduino is not None:
    arduino.close()
