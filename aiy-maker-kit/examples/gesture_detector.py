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

DEBUG_MODE = True

# Lowered confidence to detect waves in varied lighting/angles
CONFIDENCE_THRESHOLD = 0.25  
WAVE_HISTORY_LEN = 6
WAVE_MOVEMENT_THRESH = 0.15 
WAVE_SIGN_CHANGES = 2
WAVE_COOLDOWN = 2.0

# How many consecutive frames to send the signal to ensure C++ sees it
SIGNAL_REPEAT_FRAMES = 15 

# ==============================
# Global State
# ==============================
wrist_x_history = []
last_wave_time = 0.0
frames_remaining_to_send = 0
current_wrist_coords = None

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

    # Debug low confidence
    if DEBUG_MODE and (time.time() % 1.0 < 0.1):
        if r_wrist[2] < CONFIDENCE_THRESHOLD:
            print(f"[DEBUG] Low Conf: Wrist {r_wrist[2]:.2f}", flush=True)

    if r_wrist[2] < CONFIDENCE_THRESHOLD or r_elbow[2] < CONFIDENCE_THRESHOLD:
        wrist_x_history.clear()
        return None

    # Check height: Wrist should not be significantly below elbow
    if r_wrist[0] > r_elbow[0] + 0.15: 
        if DEBUG_MODE and len(wrist_x_history) > 0:
            print("[DEBUG] Wrist too low", flush=True)
        wrist_x_history.clear()
        return None

    wrist_x_history.append(r_wrist[1])
    wrist_x_history = wrist_x_history[-WAVE_HISTORY_LEN:]

    if len(wrist_x_history) >= 4:
        deltas = np.diff(wrist_x_history)
        sign_changes = np.sum(np.sign(deltas[:-1]) != np.sign(deltas[1:]))
        total_motion = np.abs(deltas).sum()

        if sign_changes >= WAVE_SIGN_CHANGES and total_motion > WAVE_MOVEMENT_THRESH:
            wrist_x_history.clear()
            return "hand_wave"
            
    return None

# ==============================
# Main Loop
# ==============================
def main():
    global last_wave_time, frames_remaining_to_send, current_wrist_coords
    
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
            # 1. Receive Header
            header = recvall(sock, 8)
            if not header: break
            width, height = struct.unpack("ii", header)

            # 2. Receive Data
            frame_size = width * height * 3
            data = recvall(sock, frame_size)
            if not data: break

            # 3. Inference
            img = Image.frombytes("RGB", (width, height), data)
            common.set_input(interpreter, img.resize(input_size))
            interpreter.invoke()
            pose = common.output_tensor(interpreter, 0).copy().reshape(17, 3)

            # 4. Logic
            now = time.time()
            gesture = detect_wave_gesture(pose)
            
            # Start repeating signal if new wave detected
            if gesture and (now - last_wave_time) > WAVE_COOLDOWN:
                last_wave_time = now
                frames_remaining_to_send = SIGNAL_REPEAT_FRAMES
                # Store coords to keep looking at hand while waving
                wrist = pose[10]
                current_wrist_coords = (wrist[0], wrist[1], wrist[2]) 
                print(f"[SEND] >>> WAVE DETECTED (Starting Sequence) <<<", flush=True)

            # Send response if we are in a "sending" window
            response_sent = False
            if frames_remaining_to_send > 0:
                frames_remaining_to_send -= 1
                
                y, x, conf = current_wrist_coords
                # Ensure box is valid (0-1 range)
                msg = f"hand_wave {conf:.2f} {x-0.1:.2f} {y-0.1:.2f} {x+0.1:.2f} {y+0.1:.2f}\n"
                
                payload = msg.encode("utf-8")
                sock.sendall(struct.pack("I", len(payload)) + payload)
                response_sent = True
                if frames_remaining_to_send == 0:
                    print(f"[INFO] Wave sequence complete.", flush=True)

            if not response_sent:
                sock.sendall(struct.pack("I", 0))

    except Exception as e:
        print(f"[ERROR] {e}", flush=True)
    finally:
        if sock: sock.close()
        print("[INFO] Gesture detector stopped", flush=True)

if __name__ == "__main__":
    main()