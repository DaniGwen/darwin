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
WAVE_COOLDOWN = 1.5

# ==============================
# Global State
# ==============================
wrist_x_history = []
last_wave_time = 0.0

# ==============================
# Socket Utilities
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

def recvall(sock, count):
    """
    Robustly receive exactly 'count' bytes from the socket.
    Returns None if the connection is closed.
    """
    buf = b''
    while count:
        try:
            newbuf = sock.recv(count)
            if not newbuf: return None
            buf += newbuf
            count -= len(newbuf)
        except OSError:
            return None
    return buf

# ==============================
# Gesture Logic
# ==============================
def detect_wave_gesture(keypoints):
    global wrist_x_history
    R_ELBOW, R_WRIST = 8, 10
    r_elbow, r_wrist = keypoints[R_ELBOW], keypoints[R_WRIST]

    if r_wrist[2] < CONFIDENCE_THRESHOLD or r_elbow[2] < CONFIDENCE_THRESHOLD:
        wrist_x_history.clear(); return None
    if r_wrist[0] > r_elbow[0] - 0.10: # Wrist below elbow
        wrist_x_history.clear(); return None

    wrist_x_history.append(r_wrist[1])
    wrist_x_history = wrist_x_history[-WAVE_HISTORY_LEN:]

    if len(wrist_x_history) >= 4:
        deltas = np.diff(wrist_x_history)
        sign_changes = np.sum(np.sign(deltas[:-1]) != np.sign(deltas[1:]))
        if sign_changes >= WAVE_SIGN_CHANGES and np.abs(deltas).sum() > WAVE_MOVEMENT_THRESH:
            wrist_x_history.clear()
            return "hand_wave"
    return None

# ==============================
# Main Loop
# ==============================
def main():
    global last_wave_time
    interpreter = make_interpreter(MODEL_PATH)
    interpreter.allocate_tensors()
    input_size = common.input_size(interpreter)

    sock = None
    while sock is None:
        sock = connect_to_cpp_server()
        time.sleep(1)

    print("[INFO] Gesture Detector Running", flush=True)

    try:
        while True:
            # 1. Receive Header (8 bytes: width, height)
            header = recvall(sock, 8)
            if not header:
                print("[WARN] C++ disconnected", flush=True)
                break

            width, height = struct.unpack("ii", header)

            # 2. Receive Image Data
            frame_size = width * height * 3
            data = recvall(sock, frame_size)
            if not data:
                print("[WARN] Incomplete frame data", flush=True)
                break

            # 3. Process Image
            img = Image.frombytes("RGB", (width, height), data)
            common.set_input(interpreter, img.resize(input_size))
            interpreter.invoke()
            pose = common.output_tensor(interpreter, 0).copy().reshape(17, 3)

            # 4. Detect & Respond
            gesture = detect_wave_gesture(pose)
            now = time.time()
            response_sent = False

            if gesture and (now - last_wave_time) > WAVE_COOLDOWN:
                last_wave_time = now
                wrist = pose[10]
                # Format: Label Score Top Left Bottom Right
                msg = f"{gesture} {wrist[2]:.2f} {wrist[1]-0.1:.2f} {wrist[0]-0.1:.2f} {wrist[1]+0.1:.2f} {wrist[0]+0.1:.2f}\n"
                
                payload = msg.encode("utf-8")
                sock.sendall(struct.pack("I", len(payload)) + payload)
                print(f"[SEND] {msg.strip()}", flush=True)
                response_sent = True

            # ALWAYS send a response (even if empty) to keep C++ synchronized
            if not response_sent:
                sock.sendall(struct.pack("I", 0))

    except Exception as e:
        print(f"[ERROR] {e}", flush=True)
    finally:
        if sock: sock.close()
        print("[INFO] Gesture detector stopped", flush=True)

if __name__ == "__main__":
    main()