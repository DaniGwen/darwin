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
WAVE_HISTORY_LEN = 12         
WAVE_MOTION_THRESHOLD = 0.20  
WAVE_COOLDOWN = 3.0           
SIGNAL_REPEAT_FRAMES = 30  

# ==============================
# Global State
# ==============================
wrist_x_history = []
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

def detect_wave_gesture(keypoints):
    global wrist_x_history
    R_WRIST_IDX = 10
    L_WRIST_IDX = 9
    NOSE_IDX = 0
    
    nose = keypoints[NOSE_IDX]
    wrist = keypoints[R_WRIST_IDX] if keypoints[R_WRIST_IDX][2] > keypoints[L_WRIST_IDX][2] else keypoints[L_WRIST_IDX]

    # ANTI-GHOST: Must clearly see a human nose (>0.4) and a wrist (>0.4)
    if nose[2] < 0.4 or wrist[2] < 0.4:
        wrist_x_history.clear() # Kill ghosts instantly!
        return None

    wrist_x_history.append(wrist[1])
    wrist_x_history = wrist_x_history[-WAVE_HISTORY_LEN:]

    if len(wrist_x_history) >= WAVE_HISTORY_LEN:
        deltas = np.diff(wrist_x_history)
        
        sign_changes = np.sum(np.diff(np.sign(deltas)) != 0)
        total_motion = np.sum(np.abs(deltas))
        span = np.max(wrist_x_history) - np.min(wrist_x_history)

        if sign_changes >= 2 and total_motion > WAVE_MOTION_THRESHOLD and span > 0.15:
            wrist_x_history.clear()
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

    print("[INFO] Gesture Detector Running (Anti-Ghost Mode)", flush=True)

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

            response_sent = False
            
            if frames_remaining_to_send > 0:
                frames_remaining_to_send -= 1
                
                # LIVE TRACKING: Send the bounding box of the whole body so C++ doesn't see a jump!
                valid_kpts = [kp for kp in pose if kp[2] > 0.2]
                if valid_kpts:
                    ys = [kp[0] for kp in valid_kpts]
                    xs = [kp[1] for kp in valid_kpts]
                    ymin, ymax = max(0.0, min(ys)), min(1.0, max(ys))
                    xmin, xmax = max(0.0, min(xs)), min(1.0, max(xs))
                else:
                    xmin, ymin, xmax, ymax = 0.0, 0.0, 1.0, 1.0
                
                live_wrist = pose[10] if pose[10][2] > pose[9][2] else pose[9]
                conf = live_wrist[2]
                
                msg = f"hand_wave {conf:.2f} {xmin:.3f} {ymin:.3f} {xmax:.3f} {ymax:.3f}"
                payload = msg.encode("utf-8")
                
                sock.sendall(struct.pack("<I", len(payload)) + payload)
                response_sent = True
                
                if frames_remaining_to_send == 0:
                    print("[INFO] Wave signal end.", flush=True)
            
            if not response_sent:
                sock.sendall(struct.pack("<I", 0))

    except Exception as e:
        print(f"[ERROR] {e}", flush=True)
    finally:
        if sock: sock.close()
        print("[INFO] Gesture detector stopped", flush=True)

if __name__ == "__main__":
    main()