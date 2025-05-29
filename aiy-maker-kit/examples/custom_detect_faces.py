#!/usr/bin/env python3

# Copyright 2021 Google LLC (Original Apache License from custom_detect_objects.py)
# Modifications for face detection by the user/AI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Performs face detection by communicating with a C++ program via Unix Domain Socket.
Receives raw image data, performs inference, and sends detection results back.
Adapted from custom_detect_objects.py.
"""

import sys
import os
import socket
import struct # For packing/unpacking integers
import time
import numpy as np
from PIL import Image

# Import tflite_runtime for the interpreter
from tflite_runtime.interpreter import Interpreter
# Import pycoral utilities for Edge TPU
from pycoral.utils.edgetpu import make_interpreter
# models.py should be in the same directory or Python path
import models # This should define FACE_DETECTION_MODEL

# --- Socket Configuration ---
SOCKET_PATH = "/tmp/darwin_detector.sock" # Ensure C++ program uses the same socket
SOCKET_CONNECT_RETRIES = 10
SOCKET_RETRY_DELAY_SEC = 0.8

# --- Model Configuration ---
# Use the FACE_DETECTION_MODEL from models.py
MODEL_PATH = models.FACE_DETECTION_MODEL
# Face detection models typically output a single class (face).
# We don't need an extensive label file like COCO.
# We can assign a label programmatically.
FACE_LABEL = "face" # The label that will be sent to C++
DETECTION_THRESHOLD = 0.5 # Confidence threshold for detections

# --- Helper function to receive exact number of bytes ---
def recvall(sock, n):
    """Helper function to receive n bytes or return None if disconnected"""
    data = b''
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data += packet
    return data

# --- Helper function to parse detection results from output tensors ---
# This function might need adjustment based on the specific output signature
# of your FACE_DETECTION_MODEL. SSD-like models usually have:
# 0: Detection boxes (e.g., [1, num_detections, 4] -> [ymin, xmin, ymax, ymax])
# 1: Detection classes (e.g., [1, num_detections]) - For face detection, this might always be class 0.
# 2: Detection scores (e.g., [1, num_detections])
# 3: Number of detections (e.g., [1])
# It's crucial to verify this with your model (e.g., using Netron).
def get_output_tensors(interpreter):
    """Returns the output tensors for detection."""
    output_details = interpreter.get_output_details()
    
    # Common output tensor indices for SSD models (verify with your face model)
    # If your face model is different, these indices might need to change.
    # Example: Some face models might only have boxes and scores if there's only one class.
    
    # Assuming a typical detection model output structure:
    # boxes: [ymin, xmin, ymax, xmax]
    # classes: class_id (often 0 for the primary detected class in single-class models)
    # scores: confidence score
    # count: number of valid detections
    
    # Find tensors by common names or typical order if names are not standard
    # This is a more robust way if tensor order can vary slightly.
    # However, for simplicity and based on custom_detect_objects.py, we'll use indexed access.
    # You might need to inspect your specific face model's output tensor details.

    try:
        boxes = interpreter.get_tensor(output_details[0]['index'])[0]   # Bounding boxes
        # For face detection, class_id might not be explicitly needed if it's always 'face'
        # but some models still output it.
        classes = interpreter.get_tensor(output_details[1]['index'])[0] # Class IDs
        scores = interpreter.get_tensor(output_details[2]['index'])[0]  # Scores
        count = int(interpreter.get_tensor(output_details[3]['index'])[0]) # Number of detections
        return boxes, classes, scores, count
    except IndexError:
        print("ERROR: Output tensor indexing issue. The face detection model's output signature "
              "might differ from the expected 4 tensors (boxes, classes, scores, count). "
              "Please verify the model structure.", file=sys.stderr)
        # Fallback for models that might only output boxes and scores (e.g., some optimized face detectors)
        # This is a guess; you MUST verify your model's output.
        try:
            print("INFO: Attempting fallback to 2-tensor output (boxes, scores) for face model.", file=sys.stderr)
            boxes = interpreter.get_tensor(output_details[0]['index'])[0]
            scores = interpreter.get_tensor(output_details[1]['index'])[0]
            # If classes and count are not present, we might have to infer count from scores
            # or assume all returned boxes are valid above a threshold.
            # For this example, we'll assume scores array length can determine potential detections.
            # The filtering by DETECTION_THRESHOLD will effectively give the count.
            # We'll assign a dummy 'classes' array if it's not present.
            num_potential_detections = scores.shape[0]
            classes_dummy = np.zeros(num_potential_detections, dtype=int) # Assign class 0 to all
            return boxes, classes_dummy, scores, num_potential_detections # Return dummy classes and inferred count
        except Exception as e:
            print(f"ERROR: Critical failure in get_output_tensors for face model: {e}", file=sys.stderr)
            return None, None, None, 0


# --- Modular Functions ---

def connect_to_socket(socket_path, retries=SOCKET_CONNECT_RETRIES, delay=SOCKET_RETRY_DELAY_SEC):
    """Connects to the Unix Domain Socket server with retries."""
    print(f"INFO: Attempting to connect to socket {socket_path}...", file=sys.stderr)
    for i in range(retries):
        try:
            client_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            client_socket.connect(socket_path)
            print("INFO: Successfully connected to C++ program.", file=sys.stderr)
            return client_socket
        except (socket.error, FileNotFoundError) as e:
            print(f"WARNING: Connection attempt {i+1}/{retries} failed: {e}. Retrying in {delay} seconds...", file=sys.stderr)
            time.sleep(delay)
        except Exception as e:
            print(f"ERROR: An unexpected error occurred during connection attempt {i+1}/{retries}: {e}", file=sys.stderr)
            time.sleep(delay)

    print(f"ERROR: Failed to connect to socket {socket_path} after {retries} attempts.", file=sys.stderr)
    return None

def load_model(model_path):
    """Loads the TFLite model for face detection."""
    try:
        print(f"INFO: Loading face detection model from: {model_path}", file=sys.stderr)
        interpreter = make_interpreter(model_path)
        interpreter.allocate_tensors()

        input_details = interpreter.get_input_details()[0]
        input_shape = input_details['shape']
        model_input_height = input_shape[1]
        model_input_width = input_shape[2]
        input_type = input_details['dtype']

        print(f"INFO: Face model expects input HxW: {model_input_height}x{model_input_width}, Type: {input_type}", file=sys.stderr)
        return interpreter, model_input_width, model_input_height, input_type

    except FileNotFoundError:
        print(f"Error: Model file not found at {model_path}", file=sys.stderr)
        return None, None, None, None
    except Exception as e:
        print(f"Error loading face detection model: {e}", file=sys.stderr)
        return None, None, None, None

def process_frame(client_socket, interpreter, model_input_width, model_input_height, input_type):
    """Receives a frame, performs face detection, and sends results."""
    width_data = recvall(client_socket, 4)
    if not width_data:
        print("INFO: Connection closed by C++ program (while receiving width).", file=sys.stderr)
        return False

    frame_width = struct.unpack('<I', width_data)[0]

    height_data = recvall(client_socket, 4)
    if not height_data:
        print("INFO: Connection closed by C++ program (while receiving height).", file=sys.stderr)
        return False

    frame_height = struct.unpack('<I', height_data)[0]
    frame_data_size = frame_width * frame_height * 3 # Assuming RGB

    raw_frame_data = recvall(client_socket, frame_data_size)
    if not raw_frame_data:
        print("INFO: Connection closed by C++ program (while receiving frame data).", file=sys.stderr)
        return False

    image_numpy_array = np.frombuffer(raw_frame_data, dtype=np.uint8).reshape((frame_height, frame_width, 3))
    pil_image = Image.fromarray(image_numpy_array, 'RGB')

    try:
        _resample_filter = Image.Resampling.LANCZOS
    except AttributeError:
        _resample_filter = Image.LANCZOS

    resized_image = pil_image.resize((model_input_width, model_input_height), _resample_filter)
    input_data = np.array(resized_image)

    if input_type == np.uint8:
        pass
    elif input_type == np.float32:
         input_data = input_data.astype(np.float32) / 255.0
    else:
         print(f"WARNING: Unsupported input tensor type for face model: {input_type}", file=sys.stderr)
         return True # Skip this frame

    input_data = np.expand_dims(input_data, axis=0)
    interpreter.set_tensor(interpreter.get_input_details()[0]['index'], input_data)
    interpreter.invoke()

    boxes, _, scores, count = get_output_tensors(interpreter) # We ignore 'classes' for face detection
    if boxes is None: # Critical error in get_output_tensors
        return True # Try to continue, but likely model issue

    detection_results_string = ""
    # The 'count' from SSD models is the number of *potential* detections.
    # We iterate up to this count and filter by score.
    # For some face models, 'count' might represent actual detections above an internal threshold.
    # It's safer to iterate through all available scores/boxes if count is very large.
    num_to_iterate = min(count, len(scores))


    for i in range(num_to_iterate):
        if scores[i] >= DETECTION_THRESHOLD:
            # For face detection, the label is always "face"
            label = FACE_LABEL
            ymin, xmin, ymax, xmax = boxes[i]
            detection_results_string += f"{label} {scores[i]} {xmin} {ymin} {xmax} {ymax}\n"

    if detection_results_string:
        detection_results_string = detection_results_string.rstrip('\n')

    result_bytes = detection_results_string.encode('utf-8')
    result_size = len(result_bytes)
    size_header = struct.pack('<I', result_size)

    try:
        client_socket.sendall(size_header)
        if result_size > 0:
            client_socket.sendall(result_bytes)
    except socket.error as e:
        print(f"ERROR: Failed to send face detection results: {e}", file=sys.stderr)
        return False
    return True

def main():
    """Main function for the face detector script."""
    client_socket = connect_to_socket(SOCKET_PATH)
    if client_socket is None:
        sys.exit(1)

    interpreter, model_input_width, model_input_height, input_type = load_model(MODEL_PATH)
    if interpreter is None:
        client_socket.close()
        sys.exit(1)

    print("INFO: Entering main face detection loop...", file=sys.stderr)
    try:
        while True:
            if not process_frame(client_socket, interpreter, model_input_width, model_input_height, input_type):
                print("INFO: Face detection loop terminated due to error or connection close.", file=sys.stderr)
                break
    except Exception as e:
        print(f"An error occurred during face processing loop: {e}", file=sys.stderr)
    finally:
        print("INFO: Closing socket connection (face detector).", file=sys.stderr)
        client_socket.close()
    sys.exit(0)

if __name__ == "__main__":
    main()