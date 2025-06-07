#!/usr/bin/env python3

# Copyright 2021 Google LLC
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
Performs object detection by communicating with a C++ program via Unix Domain Socket.
Receives raw image data, performs inference, and sends detection results back.
Refactored for modularity and added retry logic for socket connection.
"""

import sys
import os
import socket
import struct  # For packing/unpacking integers
import time
import numpy as np
from playsound import playsound
from PIL import Image

# Import tflite_runtime for the interpreter
from tflite_runtime.interpreter import Interpreter

# Import pycoral utilities for Edge TPU
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils import dataset
import models

# --- Socket Configuration ---
SOCKET_PATH = "/tmp/darwin_detector.sock"
SOCKET_CONNECT_RETRIES = 10  # Number of times to retry socket connection
SOCKET_RETRY_DELAY_SEC = 0.8  # Delay between socket connection retries in seconds

# --- Model Configuration ---
# Use the model path from models.py
MODEL_PATH = models.OBJECT_DETECTION_MODEL
# Labels path is often MODEL_PATH with .tflite replaced by .txt
LABELS_PATH = "/home/darwin/darwin/aiy-maker-kit/examples/models/coco_labels.txt"
DETECTION_THRESHOLD = 0.5  # Use the threshold you want
# MAX_DETECTIONS = 10 # The model outputs a fixed number, we'll filter by threshold


# --- Helper function to load labels ---
# Modified to always return a dictionary {id: label}
def load_labels(path):
    """Loads the labels file, returning a dictionary {id: label}."""
    print(f"DEBUG: Loading labels from: {path}", file=sys.stderr)  # Debug print
    try:
        with open(path, "r", encoding="utf-8") as f:
            lines = f.readlines()

        if not lines:
            print(f"DEBUG: Label file is empty: {path}", file=sys.stderr)  # Debug
            return {}

        # Check if the format is ID label (pycoral format like "0 background", "1 person")
        # or just label per line ("background", "person")
        # A simple check: see if the first non-empty line starts with a number followed by space
        first_non_empty_line = next(
            (line.strip() for line in lines if line.strip()), None
        )

        if first_non_empty_line and first_non_empty_line.split(maxsplit=1)[0].isdigit():
            # Likely ID label format, use pycoral reader
            print(
                f"DEBUG: Assuming label file format is 'ID label': {path}",
                file=sys.stderr,
            )  # Debug
            # Use dataset.read_label_file which returns a dict {id: label}
            # This function can handle the case where ID 0 is empty for background
            labels = dataset.read_label_file(path)
            print(
                f"DEBUG: Loaded {len(labels)} labels using dataset.read_label_file",
                file=sys.stderr,
            )  # Debug
            return labels
        else:
            # Assume simple label per line, index is the ID
            print(
                f"DEBUG: Assuming label file format is 'label per line': {path}",
                file=sys.stderr,
            )  # Debug
            labels = {}
            # Start index from 0. If class 0 is background and first line is empty,
            # the empty line will be skipped by .strip() but its index (0) will be used.
            # If the file starts with a real label for class 0, its index (0) will be used.
            # This matches how line-by-line labels often work with models where 0 is the first class.
            for i, line in enumerate(lines):
                stripped_line = line.strip()
                if stripped_line:  # Only add non-empty lines
                    labels[i] = stripped_line
            print(
                f"DEBUG: Loaded {len(labels)} labels using line index as ID",
                file=sys.stderr,
            )  # Debug
            return labels

    except FileNotFoundError:
        print(f"Error: Labels file not found at {path}", file=sys.stderr)
        return {}  # Return empty dictionary on error
    except Exception as e:
        print(f"Error loading labels file {path}: {e}", file=sys.stderr)
        return {}  # Return empty dictionary on error


# --- Helper function to receive exact number of bytes ---
def recvall(sock, n):
    """Helper function to receive n bytes or return None if disconnected"""
    data = b""
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data += packet
    return data


# --- Helper function to parse detection results from output tensors ---
# This is specific to the SSD-like model architecture (like SSD MobileNet)
# Output tensors are typically:
# 0: Detection boxes (e.g., [1, num_detections, 4] -> [ymin, xmin, ymax, ymax])
# 1: Detection classes (e.g., [1, num_detections])
# 2: Detection scores (e.g., [1, num_detections])
# 3: Number of detections (e.g., [1])
def get_output_tensors(interpreter):
    """Returns the output tensors."""
    output_details = interpreter.get_output_details()
    # Adjust indices based on your specific model if necessary
    # You can inspect your model with Netron (https://netron.app/)
    # to confirm output tensor order, shape, and type.
    # Assuming standard output order: boxes, classes, scores, num_detections
    boxes = interpreter.get_tensor(output_details[0]["index"])[0]
    classes = interpreter.get_tensor(output_details[1]["index"])[0]
    scores = interpreter.get_tensor(output_details[2]["index"])[0]
    count = int(interpreter.get_tensor(output_details[3]["index"])[0])
    return boxes, classes, scores, count


# --- Modular Functions ---


def connect_to_socket(
    socket_path, retries=SOCKET_CONNECT_RETRIES, delay=SOCKET_RETRY_DELAY_SEC
):
    """Connects to the Unix Domain Socket server with retries."""
    print(f"INFO: Attempting to connect to socket {socket_path}...", file=sys.stderr)
    for i in range(retries):
        try:
            client_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            client_socket.connect(socket_path)
            print("INFO: Successfully connected to C++ program.", file=sys.stderr)
            return client_socket
        except (socket.error, FileNotFoundError) as e:
            print(
                f"WARNING: Connection attempt {i+1}/{retries} failed: {e}. Retrying in {delay} seconds...",
                file=sys.stderr,
            )
            time.sleep(delay)
        except Exception as e:
            print(
                f"ERROR: An unexpected error occurred during connection attempt {i+1}/{retries}: {e}",
                file=sys.stderr,
            )
            time.sleep(delay)

    print(
        f"ERROR: Failed to connect to socket {socket_path} after {retries} attempts.",
        file=sys.stderr,
    )
    return None


def load_model_and_labels(model_path, labels_path):
    """Loads the TFLite model and labels."""
    try:
        interpreter = make_interpreter(model_path)
        interpreter.allocate_tensors()

        # Get model input details
        input_details = interpreter.get_input_details()[0]
        input_shape = input_details[
            "shape"
        ]  # Expected input shape (1, height, width, channels)
        model_input_height = input_shape[1]
        model_input_width = input_shape[2]
        model_input_channels = input_shape[3]
        input_type = input_details["dtype"]  # Expected input data type (e.g., uint8)

        print(
            f"INFO: Model expects input HxWxC: {model_input_height}x{model_input_width}x{model_input_channels}",
            file=sys.stderr,
        )

        # Load labels
        labels = load_labels(labels_path)
        if not labels:
            print(
                "WARNING: No labels loaded. Detections will only show class IDs.",
                file=sys.stderr,
            )

        return interpreter, labels, model_input_width, model_input_height, input_type

    except FileNotFoundError:
        print(f"Error: Model file not found at {model_path}", file=sys.stderr)
        return None, None, None, None, None
    except Exception as e:
        print(f"Error loading model or labels: {e}", file=sys.stderr)
        return None, None, None, None, None


def process_frame(
    client_socket,
    interpreter,
    labels,
    model_input_width,
    model_input_height,
    input_type,
):
    """Receives a frame, performs detection, and sends results."""
    # --- Receive Frame Dimensions ---
    width_data = recvall(client_socket, 4)
    if not width_data:  # Connection closed
        print("INFO: Connection closed by C++ program.", file=sys.stderr)
        return False  # Indicate connection closed

    frame_width = struct.unpack("<I", width_data)[
        0
    ]  # '<I' is little-endian unsigned int

    height_data = recvall(client_socket, 4)
    if not height_data:  # Connection closed
        playsound("/home/darwin/darwin/Data/mp3/Bye bye.mp3")
        print("INFO: Connection closed by C++ program.", file=sys.stderr)
        return False  # Indicate connection closed

    frame_height = struct.unpack("<I", height_data)[0]

    frame_data_size = frame_width * frame_height * 3  # Assuming 3 channels (RGB)

    # --- Receive Raw Frame Data ---
    raw_frame_data = recvall(client_socket, frame_data_size)
    if not raw_frame_data:  # Connection closed
        playsound("/home/darwin/darwin/Data/mp3/Bye bye.mp3")
        print("INFO: Connection closed by C++ program.", file=sys.stderr)
        return False  # Indicate connection closed

    # --- Process Frame Data ---
    # Convert raw bytes to numpy array
    # Assuming C++ sends RGB, uint8
    image_numpy_array = np.frombuffer(raw_frame_data, dtype=np.uint8).reshape(
        (frame_height, frame_width, 3)
    )

    # Convert numpy array to PIL Image for resizing
    pil_image = Image.fromarray(image_numpy_array, "RGB")

    # Resize image to model input size using Pillow
    try:
        _resample_filter = Image.Resampling.LANCZOS
    except AttributeError:
        _resample_filter = Image.LANCZOS  # Fallback for older Pillow

    resized_image = pil_image.resize(
        (model_input_width, model_input_height), _resample_filter
    )

    # Convert resized image back to numpy array and prepare for model input
    input_data = np.array(resized_image)

    # Ensure input data type matches model (usually uint8)
    if input_type == np.uint8:
        pass  # Data is already uint8 [0, 255]
    elif input_type == np.float32:
        input_data = input_data.astype(np.float32) / 255.0  # Example normalization
    else:
        print(f"WARNING: Unsupported input tensor type: {input_type}", file=sys.stderr)
        # Handle or skip frame if type is unexpected
        return True  # Continue loop, but skip processing this frame

    # Add batch dimension
    input_data = np.expand_dims(input_data, axis=0)

    # --- Copy Image Data to Input Tensor ---
    interpreter.set_tensor(interpreter.get_input_details()[0]["index"], input_data)

    # --- Run Inference ---
    interpreter.invoke()

    # --- Get and Parse Output Tensors ---
    boxes, classes, scores, count = get_output_tensors(interpreter)

    # --- Format Detection Results ---
    detection_results_string = ""
    # Filter by threshold and count
    for i in range(count):
        if scores[i] >= DETECTION_THRESHOLD:
            # Get label from class ID
            class_id = int(classes[i])
            # Use labels dictionary, fallback to ID if not found
            label = labels.get(class_id, f"ID_{class_id}")

            # Get bounding box coordinates (already normalized)
            ymin, xmin, ymax, xmax = boxes[i]

            # Append to the result string
            detection_results_string += (
                f"{label} {scores[i]} {xmin} {ymin} {xmax} {ymax}\n"
            )

    # Remove trailing newline if any detections were added
    if detection_results_string:
        detection_results_string = detection_results_string.rstrip("\n")

    # --- Send Detection Results Back to C++ ---
    # Send the size of the string first (as 4-byte integer)
    result_bytes = detection_results_string.encode("utf-8")
    result_size = len(result_bytes)
    size_header = struct.pack("<I", result_size)  # '<I' is little-endian unsigned int

    try:
        client_socket.sendall(size_header)
        if result_size > 0:
            client_socket.sendall(result_bytes)
    except socket.error as e:
        print(f"ERROR: Failed to send detection results: {e}", file=sys.stderr)
        return False  # Indicate send error

    return True  # Indicate successful processing and sending


def main():
    """Main function for the detector script."""
    # --- Connect to C++ Socket Server ---
    # The connect_to_socket function now handles retries
    client_socket = connect_to_socket(SOCKET_PATH)
    if client_socket is None:
        print(
            "ERROR: Could not connect to C++ program after multiple retries. Exiting.",
            file=sys.stderr,
        )
        sys.exit(1)

    # --- Load Model and Create Interpreter (Done Once) ---
    interpreter, labels, model_input_width, model_input_height, input_type = (
        load_model_and_labels(MODEL_PATH, LABELS_PATH)
    )
    if interpreter is None:
        client_socket.close()
        sys.exit(1)

    # --- Main Loop: Receive Frame, Detect, Send Results ---
    print("INFO: Entering main detection loop...", file=sys.stderr)
    try:
        while True:
            if not process_frame(
                client_socket,
                interpreter,
                labels,
                model_input_width,
                model_input_height,
                input_type,
            ):
                # process_frame returns False on connection closed or send error
                print("INFO: Detection loop terminated.", file=sys.stderr)
                break
    except Exception as e:
        # Catch any other errors during the main loop
        print(f"An error occurred during processing loop: {e}", file=sys.stderr)

    finally:
        # --- Cleanup ---
        print("INFO: Closing socket connection.", file=sys.stderr)
        client_socket.close()
        # The C++ program is responsible for unlinking the socket file

    sys.exit(0)  # Indicate successful script execution (even if loop broke)


# --- Main execution block ---
if __name__ == "__main__":
    main()
