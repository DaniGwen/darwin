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
Performs object detection on a single image file provided via command line,
and outputs detection results to standard output.
Modified from original aiy-maker-kit detect_objects.py.
"""

import sys
# Import Pillow for image loading
from PIL import Image
# Import numpy to work with image data as arrays
import numpy as np
# Keep necessary aiymakerkit and models imports
from aiymakerkit import vision
from aiymakerkit import utils
import models
# Import os to check file existence
import os

# --- Class to simulate the Frame object expected by Detector ---
# The original script's detector.get_objects expects a 'frame' object
# from vision.get_frames(). This object usually has a .array attribute
# containing the image data (e.g., as a numpy array). We'll create a
# dummy class to wrap our loaded image data.
class SimulatedFrame:
    def __init__(self, numpy_array):
        self.array = numpy_array
        # Add width and height attributes as they might be accessed
        self.width = numpy_array.shape[1]
        self.height = numpy_array.shape[0]
        # *** Add the .shape attribute required by the error ***
        self.shape = numpy_array.shape

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
        # Load the image using Pillow
        # Convert to RGB to ensure 3 channels, as expected by most models
        pil_image = Image.open(image_path).convert('RGB')

        # Convert the Pillow image to a NumPy array (uint8 is typical)
        # This is likely the format expected by the AIY vision library internally
        image_numpy_array = np.array(pil_image)

        # Create a simulated frame object from the numpy array
        simulated_frame = SimulatedFrame(image_numpy_array)

        # --- Initialize Detector and Labels (only done once per script execution) ---
        # These lines are outside the old camera loop and should remain.
        # They will run once when the script starts.
        detector = vision.Detector(models.OBJECT_DETECTION_MODEL)
        labels = utils.read_labels_from_metadata(models.OBJECT_DETECTION_MODEL)

        # --- Perform Object Detection on the single loaded image ---
        # Use the detector on the simulated frame
        # Keep the threshold or make it configurable if needed
        objects = detector.get_objects(simulated_frame, threshold=0.4)

        # --- Output Detection Results to Standard Output ---
        # Remove the original vision.draw_objects(frame, objects, labels)

        # Print results in a parseable format for the C++ program
        # Format: label score xmin ymin xmax ymax (normalized 0-1)
        # Print each detection on a new line.
        for obj in objects:
            # Get label from object ID, use 'unknown' if not found in labels
            label = labels.get(obj.id, 'unknown')
            bbox = obj.bounding_box # Bounding box is expected to have xmin, ymin, xmax, ymax (normalized)

            # Print the formatted output
            print(f"{label} {obj.score} {bbox.xmin} {bbox.ymin} {bbox.xmax} {bbox.ymax}")

        # Flush the standard output buffer
        # This is important to ensure the C++ program receives the output immediately
        sys.stdout.flush()

    except Exception as e:
        # Catch any errors during image loading or processing
        print(f"An error occurred during processing: {e}", file=sys.stderr)
        sys.exit(1)

    # The script will automatically exit after the last line is executed.
    # sys.exit(0) # Explicitly indicate success if needed