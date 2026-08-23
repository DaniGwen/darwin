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

# Tuning
CONFIDENCE_THRESHOLD = 0.45   # Increased from 0.15 to banish ghost hands!
WAVE_HISTORY_LEN = 10         
WAVE_MOTION_THRESHOLD = 0.08  
WAVE_COOLDOWN = 3.0           

SIGNAL_REPEAT_FRAMES = 30  

# ==============================
# Global State
# ==============================
wrist_x_history = []
last_wave_time = 0.0
frames_remaining_to_send = 0
current_wrist_coords = None

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

def detect_wave_gesture(keypoints):
    global wrist_x_history
    R_WRIST_IDX = 10
    L_WRIST_IDX = 9
    
    # Track the wrist with the highest confidence
    wrist = keypoints[R_WRIST_IDX] if keypoints[R_WRIST_IDX][2] > keypoints[L_WRIST_IDX][2] else keypoints[L_WRIST_IDX]

    if wrist[2] < CONFIDENCE_THRESHOLD:
        wrist_x_history.clear()
        return None

    wrist_x_history.append(wrist[1])
    wrist_x_history = wrist_x_history[-WAVE_HISTORY_LEN:]

    if len(wrist_x_history) >= WAVE_HISTORY_LEN:
        deltas = np.diff(wrist_x_history)
        
        # New Logic: Count how many times the wrist changes direction (left vs right)
        sign_changes = np.sum(np.diff(np.sign(deltas)) != 0)
        total_motion = np.sum(np.abs(deltas))
        span = np.max(wrist_x_history) - np.min(wrist_x_history)

        # Require at least 2 direction changes to prevent triggering when just raising the hand
        if sign_changes >= 2 and total_motion > WAVE_MOTION_THRESHOLD and span > 0.05:
            wrist_x_history.clear()
            return "hand_wave"

    return None

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
            header = recvall(sock, 8)
            if not header: break
            width, height = struct.unpack("ii", header)

            frame_size = width * height * 3
            data = recvall(sock, frame_size)
            if not data: break

            img = Image.frombytes("RGB", (width, height), data)
            common.set_input(interpreter, img.resize(input_size))
            interpreter.invoke()
            pose = common.output_tensor(interpreter, 0).copy().reshape(17, 3)

            now = time.time()
            gesture = detect_wave_gesture(pose)
            
            if gesture and (now - last_wave_time) > WAVE_COOLDOWN:
                last_wave_time = now
                frames_remaining_to_send = SIGNAL_REPEAT_FRAMES
                current_wrist_coords = (pose[10][0], pose[10][1], pose[10][2])
                print(f"\n[SEND] >>> WAVE DETECTED! <<<\n", flush=True)

            response_sent = False
            
            if frames_remaining_to_send > 0:
                frames_remaining_to_send -= 1
                y, x, conf = current_wrist_coords
                
                # FIXED: Send normalized 0.0-1.0 floats so C++ math doesn't explode
                xmin = max(0.0, x - 0.05)
                ymin = max(0.0, y - 0.05)
                xmax = min(1.0, x + 0.05)
                ymax = min(1.0, y + 0.05)
                
                msg = f"hand_wave {conf:.2f} {xmin:.3f} {ymin:.3f} {xmax:.3f} {ymax:.3f}"
                
                payload = msg.encode("utf-8")
                sock.sendall(struct.pack("I", len(payload)) + payload)
                response_sent = True
                
                if frames_remaining_to_send == 0:
                    print(f"[INFO] Wave signal end.", flush=True)
            
            if not response_sent:
                sock.sendall(struct.pack("I", 0))

    except Exception as e:
        print(f"[ERROR] {e}", flush=True)
    finally:
        if sock: sock.close()
        print("[INFO] Gesture detector stopped", flush=True)

if __name__ == "__main__":
    main()