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
MODEL_PATH = models.FACE_DETECTION_MODEL
FACE_LABEL = "face" 
DETECTION_THRESHOLD = 0.5 

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
# Updated based on Netron output:
# Output 0 (boxes): tensor: float32[1,50,4] (Netron ID 6)
# Output 1 (classes): tensor: float32[1,50] (Netron ID 7)
# Output 2 (scores): tensor: float32[1,50] (Netron ID 8)
# Output 3 (num_detections): tensor: float32[1] (Netron ID 9)
# The tflite_runtime interpreter.get_output_details() usually orders these logically.
def get_output_tensors(interpreter):
    """Returns the output tensors for detection."""
    output_details = interpreter.get_output_details()

    # Expected shapes based on Netron output (batch_size=1, max_detections=50)
    # We assume the interpreter provides these in the standard order.
    # If not, a more complex mapping by tensor name or exact shape would be needed.
    
    expected_shapes = {
        "boxes": (1, 50, 4),  # Or your model's max detections if not 50
        "classes": (1, 50),
        "scores": (1, 50),
        "count": (1,)
    }

    try:
        # Assuming the standard order of output tensors:
        # 0: Detection boxes
        # 1: Detection classes
        # 2: Detection scores
        # 3: Number of detections

        boxes_tensor_index = output_details[0]['index']
        classes_tensor_index = output_details[1]['index']
        scores_tensor_index = output_details[2]['index']
        count_tensor_index = output_details[3]['index']
        
        # Validate shapes (optional but good for robustness)
        # if tuple(interpreter.get_tensor(boxes_tensor_index).shape) != expected_shapes["boxes"]:
        #     print(f"Warning: Boxes tensor shape mismatch. Expected {expected_shapes['boxes']}, Got {interpreter.get_tensor(boxes_tensor_index).shape}", file=sys.stderr)
        # Similar checks for classes, scores, count...

        boxes = interpreter.get_tensor(boxes_tensor_index)[0] # Remove batch dim
        classes = interpreter.get_tensor(classes_tensor_index)[0] # Remove batch dim
        scores = interpreter.get_tensor(scores_tensor_index)[0] # Remove batch dim
        count = int(interpreter.get_tensor(count_tensor_index)[0]) # Remove batch dim and convert to int

        return boxes, classes, scores, count
        
    except IndexError as e:
        print(f"ERROR: Output tensor indexing issue in get_output_tensors. Expected 4 output tensors. Error: {e}", file=sys.stderr)
        print(f"Number of output tensors found: {len(output_details)}", file=sys.stderr)
        for i, detail in enumerate(output_details):
            print(f"Output tensor {i}: name={detail['name']}, shape={detail['shape']}, dtype={detail['dtype']}", file=sys.stderr)
        return None, None, None, 0
    except Exception as e:
        print(f"ERROR: Critical failure in get_output_tensors: {e}", file=sys.stderr)
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
        interpreter = make_interpreter(model_path) # This handles EdgeTPU delegate
        interpreter.allocate_tensors()

        input_details = interpreter.get_input_details()[0]
        # Netron: name: normalized_input_image_tensor, tensor: uint8[1,320,320,3]
        if tuple(input_details['shape']) != (1, 320, 320, 3):
             print(f"WARNING: Model input shape mismatch. Expected (1,320,320,3), Got {input_details['shape']}", file=sys.stderr)

        model_input_height = input_details['shape'][1]
        model_input_width = input_details['shape'][2]
        input_type = input_details['dtype']

        if input_type != np.uint8:
            print(f"WARNING: Model input type mismatch. Expected uint8, Got {input_type}", file=sys.stderr)


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
        _resample_filter = Image.Resampling.LANCZOS # Pillow 9.0.0+
    except AttributeError:
        _resample_filter = Image.LANCZOS # Fallback for older Pillow

    # Resize to model's expected input (320x320 based on Netron)
    resized_image = pil_image.resize((model_input_width, model_input_height), _resample_filter)
    input_data = np.array(resized_image) # Should be (320, 320, 3) uint8

    # Model expects uint8, no normalization needed if input_type is np.uint8
    if input_type != np.uint8:
         print(f"WARNING: Input type mismatch during processing. Expected uint8, got {input_type}. Attempting conversion or check model.", file=sys.stderr)
         if input_type == np.float32: # Example if model expected float
             input_data = input_data.astype(np.float32) / 255.0
         else: # If model expects something else, this might fail or be incorrect
             input_data = input_data.astype(input_type)


    input_data = np.expand_dims(input_data, axis=0) # Add batch dimension -> (1, 320, 320, 3)
    interpreter.set_tensor(interpreter.get_input_details()[0]['index'], input_data)
    interpreter.invoke()

    boxes, classes, scores, count = get_output_tensors(interpreter)
    if boxes is None: 
        print("ERROR: Failed to get output tensors. Skipping frame.", file=sys.stderr)
        # Send empty result
        size_header = struct.pack('<I', 0)
        try:
            client_socket.sendall(size_header)
        except socket.error as e:
            print(f"ERROR: Failed to send empty detection results: {e}", file=sys.stderr)
            return False
        return True


    detection_results_string = ""
    # 'count' is the number of actual detections.
    # The tensors for boxes, classes, scores are sized for max_detections (50 in this case).
    # So, we only need to iterate up to 'count'.
    for i in range(count): # Iterate up to the number of valid detections
        if scores[i] >= DETECTION_THRESHOLD:
            label = FACE_LABEL # For face detection, the label is always "face"
            
            # Boxes are [ymin, xmin, ymax, xmax]
            ymin, xmin, ymax, xmax = boxes[i]
            
            # Class ID from classes[i] is likely 0.0 for face, but we don't use it for the label.
            # class_id = int(classes[i]) 

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
    except KeyboardInterrupt:
        print("INFO: Face detection script interrupted by user (Ctrl+C).", file=sys.stderr)
    except Exception as e:
        print(f"An error occurred during face processing loop: {e}", file=sys.stderr)
    finally:
        print("INFO: Closing socket connection (face detector).", file=sys.stderr)
        client_socket.close()
    sys.exit(0)

if __name__ == "__main__":
    main()