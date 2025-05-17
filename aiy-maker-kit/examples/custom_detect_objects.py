# Copyright 2021 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Performs object detection on a single image file using tflite_runtime and pycoral,
and outputs detection results to standard output.
Bypasses aiymakerkit.vision for image loading/processing.
"""
# --- Temporary Debugging Lines ---
import sys
import PIL
try:
    print(f"DEBUG: Python Executable: {sys.executable}", file=sys.stderr)
    print(f"DEBUG: Pillow Version: {PIL.__version__}", file=sys.stderr)
    print(f"DEBUG: Pillow Location: {PIL.__file__}", file=sys.stderr)
except Exception as e:
    print(f"DEBUG: Error getting PIL info: {e}", file=sys.stderr)
# --- End Temporary Debugging Lines ---

import sys
import os
import time
# Import Pillow for image loading and basic preprocessing
from PIL import Image
# Import numpy for image data manipulation
import numpy as np
# Import tflite_runtime for the interpreter
from tflite_runtime.interpreter import Interpreter
# Import pycoral utilities for Edge TPU
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils import dataset

# Assume models.py exists and defines OBJECT_DETECTION_MODEL
# from . import models # Use relative import if models.py is in the same package
# Or if models.py is just in the same directory:
import models

# --- Configuration ---
# Use the model path from models.py
MODEL_PATH = models.OBJECT_DETECTION_MODEL
LABELS_PATH = '/home/darwin/darwin/aiy-maker-kit/examples/models/coco_labels.txt'
DETECTION_THRESHOLD = 0.4 # Use the threshold you were using
# MAX_DETECTIONS = 10 # The model outputs a fixed number, we'll filter by threshold

# --- Helper function to load labels ---
def load_labels(path):
    """Loads the labels file, returning a dictionary {id: label}."""
    print(f"DEBUG: Loading labels from: {path}", file=sys.stderr) # Debug print
    try:
        with open(path, 'r', encoding='utf-8') as f:
            lines = f.readlines()

        if not lines:
            print(f"DEBUG: Label file is empty: {path}", file=sys.stderr) # Debug
            return {}

        # Check if the format is ID label (pycoral format like "0 background", "1 person")
        # or just label per line ("background", "person")
        # A simple check: see if the first non-empty line starts with a number followed by space
        first_non_empty_line = next((line.strip() for line in lines if line.strip()), None)

        if first_non_empty_line and first_non_empty_line.split(maxsplit=1)[0].isdigit():
             # Likely ID label format, use pycoral reader
             print(f"DEBUG: Assuming label file format is 'ID label': {path}", file=sys.stderr) # Debug
             # Use dataset.read_label_file which returns a dict {id: label}
             # This function can handle the case where ID 0 is empty for background
             labels = dataset.read_label_file(path)
             print(f"DEBUG: Loaded {len(labels)} labels using dataset.read_label_file", file=sys.stderr) # Debug
             return labels
        else:
             # Assume simple label per line, index is the ID
             print(f"DEBUG: Assuming label file format is 'label per line': {path}", file=sys.stderr) # Debug
             labels = {}
             # Start index from 0. If class 0 is background and first line is empty,
             # the empty line will be skipped by .strip() but its index (0) will be used.
             # If the file starts with a real label for class 0, its index (0) will be used.
             # This matches how line-by-line labels often work with models where 0 is the first class.
             for i, line in enumerate(lines):
                 stripped_line = line.strip()
                 if stripped_line: # Only add non-empty lines
                     labels[i] = stripped_line
             print(f"DEBUG: Loaded {len(labels)} labels using line index as ID", file=sys.stderr) # Debug
             return labels

    except FileNotFoundError:
        print(f"Error: Labels file not found at {path}", file=sys.stderr)
        return {} # Return empty dictionary on error
    except Exception as e:
        print(f"Error loading labels file {path}: {e}", file=sys.stderr)
        return {} # Return empty dictionary on error

# --- Helper function to parse detection results from output tensors ---
# This is specific to the SSD-like model architecture (like SSD MobileNet)
# Output tensors are typically:
# 0: Detection boxes (e.g., [1, num_detections, 4] -> [ymin, xmin, ymax, xmax])
# 1: Detection classes (e.g., [1, num_detections])
# 2: Detection scores (e.g., [1, num_detections])
# 3: Number of detections (e.g., [1])
def get_output_tensors(interpreter):
    """Returns the output tensors."""
    output_details = interpreter.get_output_details()
    # Adjust indices based on your specific model if necessary
    # You can inspect your model with Netron (https://netron.app/)
    # to confirm output tensor order, shape, and type.
    boxes = interpreter.get_tensor(output_details[0]['index'])[0]
    classes = interpreter.get_tensor(output_details[1]['index'])[0]
    scores = interpreter.get_tensor(output_details[2]['index'])[0]
    count = int(interpreter.get_tensor(output_details[3]['index'])[0])
    return boxes, classes, scores, count

# --- Main execution block ---
if __name__ == "__main__":
    # Check if an image file path was provided as a command-line argument
    if len(sys.argv) != 2:
        print("Usage: python3 detect_objects.py <image_path>", file=sys.stderr)
        sys.exit(1)

    image_path = sys.argv[1]

    # Check if the file exists
    if not os.path.exists(image_path):
        print(f"Error: Image file not found at {image_path}", file=sys.stderr)
        sys.exit(1)

    try:
        # --- Load Model and Create Interpreter ---
        # This is done once when the script starts
        interpreter = make_interpreter(MODEL_PATH)
        interpreter.allocate_tensors()

        # Get model input details
        input_details = interpreter.get_input_details()[0]
        input_shape = input_details['shape'] # Expected input shape (1, height, width, channels)
        input_height = input_shape[1]
        input_width = input_shape[2]
        input_type = input_details['dtype'] # Expected input data type (e.g., uint8)

        # Load labels
        labels = load_labels(LABELS_PATH)

        # --- Load and Preprocess the Image ---
        # Load image using Pillow
        pil_image = Image.open(image_path).convert('RGB')

        # Resize image to model input size using Pillow
        resized_image = pil_image.resize((input_width, input_height), Image.Resampling.LANCZOS)

        # Convert resized image to numpy array
        input_data = np.array(resized_image)

        # Models might expect different input types (uint8 or float32)
        # For uint8 models, data is typically [0, 255].
        # For float32 models, data is often normalized to [0, 1] or [-1, 1].
        # Check your model's requirements. Most Edge TPU models are quantized (uint8).
        if input_type == np.uint8:
            # Data is already uint8 [0, 255] from np.array, no further scaling needed for typical models.
            pass
        elif input_type == np.float32:
            # Example: Normalize uint8 [0, 255] to float32 [0, 1]
            input_data = input_data.astype(np.float32) / 255.0
            # Example: Normalize uint8 [0, 255] to float32 [-1, 1]
            # input_data = (input_data.astype(np.float32) - 127.5) / 127.5
        else:
             print(f"Error: Unsupported input tensor type: {input_type}", file=sys.stderr)
             sys.exit(1)

        # Add batch dimension (models expect input in shape [batch_size, height, width, channels])
        input_data = np.expand_dims(input_data, axis=0) # Shape becomes [1, height, width, channels]

        # --- Copy Image Data to Input Tensor ---
        interpreter.set_tensor(input_details['index'], input_data)

        # --- Run Inference ---
        interpreter.invoke()

        # --- Get and Parse Output Tensors ---
        boxes, classes, scores, count = get_output_tensors(interpreter)

        # --- Output Detection Results to Standard Output ---
        # Print results in a parseable format for the C++ program
        # Format: label score xmin ymin xmax ymax (normalized 0-1)
        # Print each detection on a new line.
        # Filter by threshold and count
        for i in range(count):
            if scores[i] >= DETECTION_THRESHOLD:
                # Get label from class ID
                class_id = int(classes[i])
                label = labels.get(class_id, 'unknown') # Get label from ID, default to 'unknown'

                # Get bounding box coordinates (already normalized)
                ymin, xmin, ymax, xmax = boxes[i]

                # Print the formatted output
                print(f"{label} {scores[i]} {xmin} {ymin} {xmax} {ymax}")

        # Important: Flush the standard output buffer
        # This is crucial to ensure the C++ program receives the output immediately
        sys.stdout.flush()

    except FileNotFoundError:
        print(f"Error: Model file not found at {MODEL_PATH}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        # Catch any other errors during processing
        print(f"An error occurred during processing: {e}", file=sys.stderr)
        sys.exit(1)

    # The script will automatically exit after the last line is executed.
    # sys.exit(0) # Explicitly indicate success if needed