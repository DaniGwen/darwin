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

# Realistic Wave Tuning
WAVE_HISTORY_LEN = 20         # ~0.7-1.0s buffer
WAVE_MOTION_THRESHOLD = 0.20  # Cumulative motion (20% of frame width)
WAVE_SPAN_THRESHOLD = 0.06    # Stride width (at least 6% of frame width)
WAVE_MIN_SIGNS = 2            # 2 direction reversals (left-right-left)
WAVE_COOLDOWN = 3.5           
SIGNAL_REPEAT_FRAMES = 20     

# Global State
right_wrist_history = []
left_wrist_history = []
last_wave_time = 0.0
frames_remaining_to_send = 0
debug_tick = 0

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

def check_single_wrist_wave(wrist, shoulder, history, name="Wrist"):
    global debug_tick

    # 1. Wrist must meet minimum confidence
    if wrist[2] < 0.25:
        if len(history) > 0:
            history.pop(0)  # Decay slowly rather than clearing instantly
        return False

    # 2. Wrist should be near or above shoulder height (y is 0 at top, 1 at bottom)
    if shoulder[2] >= 0.25 and wrist[0] > (shoulder[0] + 0.10):
        # Wrist is hanging down at waist level; not a wave
        if len(history) > 0:
            history.pop(0)
        return False

    history.append(wrist[1])
    while len(history) > WAVE_HISTORY_LEN:
        history.pop(0)

    if len(history) >= 12:
        deltas = np.diff(history)
        # Filter micro-jitter
        significant_deltas = deltas[np.abs(deltas) > 0.005]
        if len(significant_deltas) > 2:
            sign_changes = np.sum(np.diff(np.sign(significant_deltas)) != 0)
            total_motion = np.sum(np.abs(deltas))
            span = np.max(history) - np.min(history)

            if debug_tick % 15 == 0:
                print(f"[DEBUG {name}] Conf: {wrist[2]:.2f} | Hist: {len(history)} | Signs: {sign_changes}/{WAVE_MIN_SIGNS} | Mot: {total_motion:.2f}/{WAVE_MOTION_THRESHOLD} | Span: {span:.2f}/{WAVE_SPAN_THRESHOLD}", flush=True)

            if sign_changes >= WAVE_MIN_SIGNS and total_motion >= WAVE_MOTION_THRESHOLD and span >= WAVE_SPAN_THRESHOLD:
                history.clear()
                return True
            
    return False

def detect_wave_gesture(keypoints):
    global right_wrist_history, left_wrist_history
    NOSE_IDX = 0
    L_SHOULDER_IDX = 5
    R_SHOULDER_IDX = 6
    L_WRIST_IDX = 9
    R_WRIST_IDX = 10
    
    nose = keypoints[NOSE_IDX]
    if nose[2] < 0.20:
        return None

    r_wrist = keypoints[R_WRIST_IDX]
    r_shoulder = keypoints[R_SHOULDER_IDX]
    l_wrist = keypoints[L_WRIST_IDX]
    l_shoulder = keypoints[L_SHOULDER_IDX]

    r_wave = check_single_wrist_wave(r_wrist, r_shoulder, right_wrist_history, "R_Arm")
    l_wave = check_single_wrist_wave(l_wrist, l_shoulder, left_wrist_history, "L_Arm")

    if r_wave or l_wave:
        return "hand_wave"

    return None

def main():
    global last_wave_time, frames_remaining_to_send, debug_tick
    
    interpreter = make_interpreter(MODEL_PATH)
    interpreter.allocate_tensors()
    input_size = common.input_size(interpreter)

    sock = None
    while sock is None:
        sock = connect_to_cpp_server()
        time.sleep(1)

    print("[INFO] MoveNet Gesture Detector Hot & Ready", flush=True)

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

            debug_tick += 1
            now = time.time()
            gesture = detect_wave_gesture(pose)
            
            if gesture and (now - last_wave_time) > WAVE_COOLDOWN:
                last_wave_time = now
                frames_remaining_to_send = SIGNAL_REPEAT_FRAMES
                print(f"\n======================================", flush=True)
                print(f"[TRIGGER] >>> WAVE GESTURE CONFIRMED <<<", flush=True)
                print(f"======================================\n", flush=True)
                os.system('espeak "Hey! Hi!" 2>/dev/null &')

            msg = ""
            valid_kpts = [kp for kp in pose if kp[2] > 0.15]
            if valid_kpts:
                ys = [kp[0] for kp in valid_kpts]
                xs = [kp[1] for kp in valid_kpts]
                ymin, ymax = max(0.0, min(ys)), min(1.0, max(ys))
                xmin, xmax = max(0.0, min(xs)), min(1.0, max(xs))
                
                if frames_remaining_to_send > 0:
                    frames_remaining_to_send -= 1
                    live_wrist = pose[10] if pose[10][2] > pose[9][2] else pose[9]
                    msg = f"hand_wave {live_wrist[2]:.2f} {xmin:.3f} {ymin:.3f} {xmax:.3f} {ymax:.3f}"
                    if frames_remaining_to_send == 0:
                        print("[INFO] Wave signal window closed.", flush=True)
                else:
                    msg = f"person {pose[0][2]:.2f} {xmin:.3f} {ymin:.3f} {xmax:.3f} {ymax:.3f}"
            
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