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

# Gesture thresholds
CONFIDENCE_THRESHOLD = 0.15   
WAVE_HISTORY_LEN = 10         
WAVE_MOTION_THRESHOLD = 0.08  
WAVE_COOLDOWN = 3.0           # Long cooldown to prevent double triggers

# Send the signal for ~2 seconds (assuming ~20fps) to ensure C++ sees it
SIGNAL_REPEAT_FRAMES = 40 

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
    
    # Indices: 10 = Right Wrist
    R_WRIST_IDX = 10
    wrist = keypoints[R_WRIST_IDX] # [y, x, score]

    # 1. Check Confidence
    if wrist[2] < CONFIDENCE_THRESHOLD:
        wrist_x_history.clear()
        return None

    # 2. Track X-movement
    wrist_x_history.append(wrist[1])
    wrist_x_history = wrist_x_history[-WAVE_HISTORY_LEN:]

    # 3. Analyze Motion
    if len(wrist_x_history) >= WAVE_HISTORY_LEN:
        deltas = np.diff(wrist_x_history)
        total_motion = np.sum(np.abs(deltas))
        span = np.max(wrist_x_history) - np.min(wrist_x_history)

        if total_motion > WAVE_MOTION_THRESHOLD and span > 0.05:
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

    print("[INFO] Gesture Detector Running (Robust Mode)", flush=True)
    
    last_debug_time = time.time()

    try:
        while True:
            # 1. Receive Header
            header = recvall(sock, 8)
            if not header: break
            width, height = struct.unpack("ii", header)

            # 2. Receive Image Data
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
            
            # If we detect a new wave, start the broadcast sequence
            if gesture and (now - last_wave_time) > WAVE_COOLDOWN:
                last_wave_time = now
                frames_remaining_to_send = SIGNAL_REPEAT_FRAMES
                
                # Snapshot coords
                wrist = pose[10]
                current_wrist_coords = (wrist[0], wrist[1], wrist[2])
                print(f"\n[SEND] >>> WAVE DETECTED! Broadcasting for {SIGNAL_REPEAT_FRAMES} frames... <<<\n", flush=True)

            # 5. Send Response
            response_sent = False
            
            if frames_remaining_to_send > 0:
                frames_remaining_to_send -= 1
                
                # Ensure we send valid coords
                y, x, conf = current_wrist_coords
                x = max(0.0, min(1.0, x))
                y = max(0.0, min(1.0, y))
                
                # Protocol: Label Confidence X Y W H
                # We send a dummy box around the wrist
                msg = f"hand_wave {conf:.2f} {x:.2f} {y:.2f} 0.1 0.1\n"
                
                payload = msg.encode("utf-8")
                sock.sendall(struct.pack("I", len(payload)) + payload)
                response_sent = True
                
                if frames_remaining_to_send == 0:
                    print(f"[INFO] Broadcast complete.", flush=True)
            
            # If not broadcasting wave, send empty (Keep-Alive)
            if not response_sent:
                sock.sendall(struct.pack("I", 0))

    except Exception as e:
        print(f"[ERROR] {e}", flush=True)
    finally:
        if sock: sock.close()
        print("[INFO] Gesture detector stopped", flush=True)

if __name__ == "__main__":
    main()