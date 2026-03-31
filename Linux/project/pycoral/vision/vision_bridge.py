#!/usr/bin/env python3

import cv2
import os
import time
from pycoral.adapters import common
from pycoral.adapters import detect
from pycoral.utils.edgetpu import make_interpreter

# Paths to your existing AIY models
MODEL_PATH = '/home/darwin/darwin/aiy-maker-kit/projects/ssd_mobilenet_v2_coco_quant_postprocess_edgetpu.tflite'
LABELS_PATH = '/home/darwin/darwin/aiy-maker-kit/examples/models/coco_labels.txt'

# RAM file for C++ to read instantly
SHARED_MEM_FILE = '/dev/shm/vision_target.txt'

def load_labels(path):
    with open(path, 'r') as f:
        return {i: line.strip() for i, line in enumerate(f.readlines())}

def main():
    print("[INFO] Loading Edge TPU Interpreter...")
    interpreter = make_interpreter(MODEL_PATH)
    interpreter.allocate_tensors()
    labels = load_labels(LABELS_PATH)

    # Open the camera (0 is usually /dev/video0)
    cap = cv2.VideoCapture(0)
    
    print("[INFO] Camera open. Starting PyCoral detection loop...")
    
    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            # Format the image for the Edge TPU
            _, scale = common.set_resized_input(
                interpreter, frame.shape[:2], lambda size: cv2.resize(frame, size))

            # Run the inference
            interpreter.invoke()
            
            # Get the results
            objects = detect.get_objects(interpreter, score_threshold=0.5)

            target_found = False
            for obj in objects:
                label = labels.get(obj.id, obj.id)
                
                # Only track sports balls or people
                if label in ['sports ball', 'person']:
                    bbox = obj.bbox
                    center_x = int((bbox.xmin + bbox.xmax) / 2)
                    center_y = int((bbox.ymin + bbox.ymax) / 2)
                    
                    # Write to RAM
                    with open(SHARED_MEM_FILE, 'w') as f:
                        f.write(f"{label},{center_x},{center_y}\n")
                    
                    target_found = True
                    break

            if not target_found:
                with open(SHARED_MEM_FILE, 'w') as f:
                    f.write("none,0,0\n")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[INFO] Stopping vision bridge.")
    finally:
        cap.release()
        if os.path.exists(SHARED_MEM_FILE):
            os.remove(SHARED_MEM_FILE)

if __name__ == '__main__':
    main()