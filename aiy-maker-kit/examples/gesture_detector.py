import socket
import struct
import time
import numpy as np
from PIL import Image
from pycoral.adapters import common
from pycoral.adapters import detect
from pycoral.utils.edgetpu import make_interpreter
from pycoral.adapters import classify

# --- Configuration ---
SOCKET_PATH = "/tmp/darwin_detector.sock"
MODEL_PATH = "/home/darwin/darwin/aiy-maker-kit/examples/models/movenet_single_pose_lightning_ptq_edgetpu.tflite"
CONFIDENCE_THRESHOLD = 0.4

# Gesture Logic Variables
WAVE_HISTORY_LEN = 5
wrist_x_history = []
last_wave_time = 0


def connect_to_cpp_server():
    """Connects to the Unix socket server created by HeadTracking.cpp."""
    client_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        client_sock.connect(SOCKET_PATH)
        print(f"Connected to C++ server at {SOCKET_PATH}")
        return client_sock
    except Exception as e:
        print(f"Waiting for C++ server... ({e})")
        return None


def detect_wave_gesture(keypoints):
    """
    Simple logic: If wrist is above elbow and moving left-right.
    Keypoint Indices (MoveNet): 5=L_Shoulder, 6=R_Shoulder, 7=L_Elbow, 8=R_Elbow, 9=L_Wrist, 10=R_Wrist
    Returns: 'hand_wave' or None
    """
    global wrist_x_history, last_wave_time

    # MoveNet Output is [1, 17, 3] -> (y, x, score)
    # We check Right Arm (Index 6, 8, 10)
    # Note: Coordinates are normalized 0.0 to 1.0
    r_shoulder = keypoints[6]
    r_elbow = keypoints[8]
    r_wrist = keypoints[10]

    # Check Confidence
    if r_wrist[2] < CONFIDENCE_THRESHOLD or r_elbow[2] < CONFIDENCE_THRESHOLD:
        wrist_x_history = []  # Lost tracking
        return None

    # 1. Is Hand Up? (Wrist Y < Elbow Y) - Remember Y=0 is top
    if r_wrist[0] > r_elbow[0]:
        wrist_x_history = []  # Hand is down
        return None

    # 2. Track lateral movement (Waving)
    wrist_x_history.append(r_wrist[1])
    if len(wrist_x_history) > WAVE_HISTORY_LEN:
        wrist_x_history.pop(0)

    # Calculate variance (how much X is changing)
    if len(wrist_x_history) == WAVE_HISTORY_LEN:
        variance = np.var(wrist_x_history)
        # If variance is high enough, it means waving
        if variance > 0.005:
            # Debounce: Don't spam detection
            if time.time() - last_wave_time > 2.0:
                last_wave_time = time.time()
                return "hand_wave"

    return None


def main():
    interpreter = make_interpreter(MODEL_PATH)
    interpreter.allocate_tensors()

    # Connect to C++
    sock = None
    while sock is None:
        sock = connect_to_cpp_server()
        time.sleep(1)

    print("Gesture Detector Running...")

    try:
        while True:
            # 1. Receive Frame Dimensions (Width, Height)
            header_data = sock.recv(8)
            if not header_data:
                break
            width, height = struct.unpack("ii", header_data)

            # 2. Receive Image Data
            frame_size = width * height * 3
            data = b""
            while len(data) < frame_size:
                packet = sock.recv(frame_size - len(data))
                if not packet:
                    break
                data += packet

            # Convert to PIL Image for TPU
            img = Image.frombytes("RGB", (width, height), data)

            # 3. Run Inference
            resized_img = img.resize(common.input_size(interpreter), Image.ANTIALIAS)
            common.set_input(interpreter, resized_img)
            interpreter.invoke()

            # Get Keypoints
            pose = common.output_tensor(interpreter, 0).copy().reshape(17, 3)

            # 4. Check for Gesture
            detected_label = detect_wave_gesture(pose)

            # 5. Send Result back to C++
            # Format: "label score xmin ymin xmax ymax\n"
            # We fake a bounding box for the "wave" around the wrist
            if detected_label:
                # Create a small box around wrist for visualization
                wrist = pose[10]  # Right wrist
                msg = f"{detected_label} {wrist[2]:.2f} {wrist[1]-0.1:.2f} {wrist[0]-0.1:.2f} {wrist[1]+0.1:.2f} {wrist[0]+0.1:.2f}\n"
                print(f"Sending: {msg.strip()}")
            else:
                msg = "\n"  # No detection

            # Send length first, then string
            packet = msg.encode("utf-8")
            sock.sendall(struct.pack("I", len(packet)) + packet)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        if sock:
            sock.close()


if __name__ == "__main__":
    main()
