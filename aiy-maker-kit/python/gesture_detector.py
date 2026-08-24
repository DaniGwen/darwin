#!/usr/bin/env python3
import socket
import struct
import time
import os
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
WAVE_HISTORY_LEN = 12         
WAVE_MOTION_THRESHOLD = 1.5   # Raised to ignore ambient shifting
WAVE_SPAN_THRESHOLD = 0.3     # Raised to require a deliberate, wide wave
WAVE_MIN_SIGNS = 4            # Must change directions at least 4 times
WAVE_COOLDOWN = 3.0           
SIGNAL_REPEAT_FRAMES = 30  

# ==============================
# Global State
# ==============================
right_wrist_history = []
left_wrist_history = []
last_wave_time = 0.0
frames_remaining_to_send = 0

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

def check_single_wrist_wave(wrist, history):
    # STRICT CONFIDENCE FLOOR: Ignore blurry or noisy wrist data
    if wrist[2] >= 0.50:
        history.append(wrist[1])
        # Keep history to max length
        while len(history) > WAVE_HISTORY_LEN:
            history.pop(0)
    else:
        # If confidence drops, clear history to prevent noise buildup
        history.clear()
        return False

    if len(history) >= WAVE_HISTORY_LEN:
        deltas = np.diff(history)
        sign_changes = np.sum(np.diff(np.sign(deltas)) != 0)
        total_motion = np.sum(np.abs(deltas))
        span = np.max(history) - np.min(history)

        if sign_changes >= WAVE_MIN_SIGNS and total_motion > WAVE_MOTION_THRESHOLD and span > WAVE_SPAN_THRESHOLD:
            history.clear()
            return True
            
    return False

def detect_wave_gesture(keypoints):
    global right_wrist_history, left_wrist_history
    R_WRIST_IDX = 10
    L_WRIST_IDX = 9
    NOSE_IDX = 0
    
    nose = keypoints[NOSE_IDX]
    
    # ANTI-GHOST: Must clearly see a face to allow waving
    if nose[2] < 0.40:
        right_wrist_history.clear()
        left_wrist_history.clear()
        return None

    # Check right and left wrists COMPLETELY INDEPENDENTLY
    r_wrist = keypoints[R_WRIST_IDX]
    l_wrist = keypoints[L_WRIST_IDX]

    r_wave = check_single_wrist_wave(r_wrist, right_wrist_history)
    l_wave = check_single_wrist_wave(l_wrist, left_wrist_history)

    if r_wave or l_wave:
        return "hand_wave"

    return None

def main():
    global last_wave_time, frames_remaining_to_send
    
    interpreter = make_interpreter(MODEL_PATH)
    interpreter.allocate_tensors()
    input_size = common.input_size(interpreter)

    sock = None
    while sock is None:
        sock = connect_to_cpp_server()
        time.sleep(1)

    print("[INFO] Gesture Detector Running (Independent Wrists + Voice)", flush=True)

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
                print(f"\n[SEND] >>> WAVE DETECTED! <<<\n", flush=True)
                
                # --- SYNCHRONOUS VOICE TRIGGER ---
                # This will intentionally pause the script for a second to speak before triggering the physical wave
                os.system('espeak "Hey! Hi!" 2>/dev/null')

            msg = ""
            
            # Find the bounding box for the person
            valid_kpts = [kp for kp in pose if kp[2] > 0.2]
            if valid_kpts:
                ys = [kp[0] for kp in valid_kpts]
                xs = [kp[1] for kp in valid_kpts]
                ymin, ymax = max(0.0, min(ys)), min(1.0, max(ys))
                xmin, xmax = max(0.0, min(xs)), min(1.0, max(xs))
                
                if frames_remaining_to_send > 0:
                    frames_remaining_to_send -= 1
                    
                    # Grab the highest confidence wrist just to pass coordinates to C++
                    live_wrist = pose[10] if pose[10][2] > pose[9][2] else pose[9]
                    
                    msg = f"hand_wave {live_wrist[2]:.2f} {xmin:.3f} {ymin:.3f} {xmax:.3f} {ymax:.3f}"
                    
                    if frames_remaining_to_send == 0:
                        print("[INFO] Wave signal end.", flush=True)
                else:
                    msg = f"person {pose[0][2]:.2f} {xmin:.3f} {ymin:.3f} {xmax:.3f} {ymax:.3f}"
            
            # --- SEND TO C++ ---
            if msg:
                payload = msg.encode("utf-8")
                sock.sendall(struct.pack("<I", len(payload)) + payload)
            else:
                sock.sendall(struct.pack("<I", 0))

    except Exception as e:
        print(f"[ERROR] {e}", flush=True)
    finally:
        if sock: sock.close()
        print("[INFO] Gesture detector stopped", flush=True)

if __name__ == "__main__":
    main()