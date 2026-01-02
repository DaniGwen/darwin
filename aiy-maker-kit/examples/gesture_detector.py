#!/usr/bin/env python3
import socket
import struct
import time
import models
import numpy as np
from PIL import Image

from pycoral.adapters import common
from pycoral.utils.edgetpu import make_interpreter

# ==============================
# Configuration
# ==============================
SOCKET_PATH = "/tmp/darwin_detector.sock"
MODEL_PATH = models.MOVENET_MODEL

CONFIDENCE_THRESHOLD = 0.4

WAVE_HISTORY_LEN = 6
WAVE_MOVEMENT_THRESH = 0.18
WAVE_SIGN_CHANGES = 2
WAVE_COOLDOWN = 1.5  # seconds between detections

# ==============================
# Gesture State
# ==============================
wrist_x_history = []
last_wave_time = 0.0

# ==============================
# Socket Handling
# ==============================
def connect_to_cpp_server():
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        sock.connect(SOCKET_PATH)
        print(f"[INFO] Connected to C++ server at {SOCKET_PATH}", flush=True)
        return sock
    except Exception as e:
        print(f"[INFO] Waiting for C++ server... ({e})", flush=True)
        return None

# ==============================
# Gesture Detection
# ==============================
def detect_wave_gesture(keypoints):
    """
    Robust right-hand wave detection using MoveNet keypoints.
    keypoints: shape (17, 3) -> [y, x, score]
    """
    global wrist_x_history

    # MoveNet indices
    R_ELBOW = 8
    R_WRIST = 10

    r_elbow = keypoints[R_ELBOW]
    r_wrist = keypoints[R_WRIST]

    # 1. Visibility check
    if r_wrist[2] < CONFIDENCE_THRESHOLD or r_elbow[2] < CONFIDENCE_THRESHOLD:
        wrist_x_history.clear()
        return None

    # 2. Wrist must be clearly above elbow
    if r_wrist[0] > r_elbow[0] - 0.10:
        wrist_x_history.clear()
        return None

    # 3. Track horizontal motion
    wrist_x_history.append(r_wrist[1])
    wrist_x_history = wrist_x_history[-WAVE_HISTORY_LEN:]

    # 4. Detect oscillation pattern
    if len(wrist_x_history) >= 4:
        deltas = np.diff(wrist_x_history)
        sign_changes = np.sum(np.sign(deltas[:-1]) != np.sign(deltas[1:]))

        if (
            sign_changes >= WAVE_SIGN_CHANGES
            and np.abs(deltas).sum() > WAVE_MOVEMENT_THRESH
        ):
            wrist_x_history.clear()
            return "hand_wave"

    return None

# ==============================
# Main Loop
# ==============================
def main():
    global last_wave_time

    # Initialize TPU
    interpreter = make_interpreter(MODEL_PATH)
    interpreter.allocate_tensors()
    input_size = common.input_size(interpreter)

    # Connect to C++
    sock = None
    while sock is None:
        sock = connect_to_cpp_server()
        time.sleep(1)

    print("[INFO] Gesture Detector Running", flush=True)

    try:
        while True:
            # ---- Receive frame header ----
            header = sock.recv(8)
            if not header:
                print("[WARN] C++ disconnected", flush=True)
                break

            width, height = struct.unpack("ii", header)

            # ---- Receive image ----
            frame_size = width * height * 3
            data = b""
            while len(data) < frame_size:
                packet = sock.recv(frame_size - len(data))
                if not packet:
                    break
                data += packet

            if len(data) != frame_size:
                continue

            # ---- Convert image ----
            img = Image.frombytes("RGB", (width, height), data)
            resized = img.resize(input_size)
            common.set_input(interpreter, resized)

            # ---- Inference ----
            interpreter.invoke()
            pose = common.output_tensor(interpreter, 0).copy().reshape(17, 3)

            # ---- Gesture detection ----
            gesture = detect_wave_gesture(pose)
            now = time.time()
            
            response_sent = False

            if gesture and (now - last_wave_time) > WAVE_COOLDOWN:
                last_wave_time = now

                wrist = pose[10]  # Right wrist
                msg = (
                    f"{gesture} {wrist[2]:.2f} "
                    f"{wrist[1]-0.1:.2f} {wrist[0]-0.1:.2f} "
                    f"{wrist[1]+0.1:.2f} {wrist[0]+0.1:.2f}\n"
                )

                payload = msg.encode("utf-8")
                # Send size + payload
                sock.sendall(struct.pack("I", len(payload)) + payload)
                print(f"[SEND] {msg.strip()}", flush=True)
                response_sent = True

            # ---- Always send a response! ----
            # If we didn't send a detection, we MUST send a 0-length packet 
            # so the C++ code knows we finished processing the frame.
            if not response_sent:
                sock.sendall(struct.pack("I", 0))

    except Exception as e:
        print(f"[ERROR] {e}", flush=True)

    finally:
        sock.close()
        print("[INFO] Gesture detector stopped", flush=True)

if __name__ == "__main__":
    main()