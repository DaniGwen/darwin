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

# --- Tuning Parameters ---
CONFIDENCE_THRESHOLD = 0.15   # Much lower to detect distant/blurry hands
WAVE_HISTORY_LEN = 10         # Look at last 10 frames
WAVE_MOTION_THRESHOLD = 0.08  # How much X-movement constitutes a wave
WAVE_COOLDOWN = 2.0           # Seconds to wait before detecting again

# Number of frames to repeat the signal to C++ to ensure it's caught
SIGNAL_REPEAT_FRAMES = 10 

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
    """
    Simplified Wave Detection:
    Just looks for high variance/movement in the X-axis of the right wrist
    """
    global wrist_x_history
    
    # Indices: 10 = Right Wrist, 8 = Right Elbow
    R_WRIST_IDX = 10
    wrist = keypoints[R_WRIST_IDX] # [y, x, score]

    # 1. Check Confidence
    if wrist[2] < CONFIDENCE_THRESHOLD:
        if DEBUG_MODE and (time.time() % 1.0 < 0.1):
            print(f"[DEBUG] Low Confidence: {wrist[2]:.2f}", flush=True)
        wrist_x_history.clear()
        return None

    # 2. Track X-movement
    wrist_x_history.append(wrist[1])
    wrist_x_history = wrist_x_history[-WAVE_HISTORY_LEN:]

    # 3. Analyze Motion
    if len(wrist_x_history) >= WAVE_HISTORY_LEN:
        # Calculate total distance traveled in X
        deltas = np.diff(wrist_x_history)
        total_motion = np.sum(np.abs(deltas))
        
        # Calculate span (width) of the wave
        span = np.max(wrist_x_history) - np.min(wrist_x_history)

        # Debug print status every ~0.5s
        if DEBUG_MODE and (time.time() % 0.5 < 0.05):
            print(f"[DEBUG] Motion: {total_motion:.2f} (Thresh: {WAVE_MOTION_THRESHOLD}) Span: {span:.2f}", flush=True)

        # TRIGGER CONDITION:
        # Sufficient total motion AND the motion covers a wide enough area
        if total_motion > WAVE_MOTION_THRESHOLD and span > 0.05:
            wrist_x_history.clear() # Reset to avoid double counting
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

    print("[INFO] Gesture Detector Running (Sensitive Mode)", flush=True)

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
            
            # State Machine for Signal Repeating
            if gesture and (now - last_wave_time) > WAVE_COOLDOWN:
                last_wave_time = now
                frames_remaining_to_send = SIGNAL_REPEAT_FRAMES
                
                # Snapshot coords for the message
                wrist = pose[10]
                current_wrist_coords = (wrist[0], wrist[1], wrist[2])
                print(f"\n[SEND] >>> WAVE DETECTED! (Repeating signal {SIGNAL_REPEAT_FRAMES} times) <<<\n", flush=True)

            # Send response
            response_sent = False
            if frames_remaining_to_send > 0:
                frames_remaining_to_send -= 1
                
                y, x, conf = current_wrist_coords
                # Construct message expected by C++ HeadTracking (label confidence x y w h)
                # We send a dummy box around the wrist
                msg = f"hand_wave {conf:.2f} {x-0.05:.2f} {y-0.05:.2f} {x+0.05:.2f} {y+0.05:.2f}\n"
                
                payload = msg.encode("utf-8")
                sock.sendall(struct.pack("I", len(payload)) + payload)
                response_sent = True
            
            # Always send size=0 if no detection (Keep-Alive for C++)
            if not response_sent:
                sock.sendall(struct.pack("I", 0))

    except Exception as e:
        print(f"[ERROR] {e}", flush=True)
    finally:
        if sock: sock.close()
        print("[INFO] Gesture detector stopped", flush=True)

if __name__ == "__main__":
    main()