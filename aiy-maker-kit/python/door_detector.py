#!/usr/bin/env python3
import socket
import struct
import time
import numpy as np
import cv2
import cv2.aruco as aruco

SOCKET_PATH = "/tmp/darwin_detector.sock"

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

def main():
    sock = None
    while sock is None:
        sock = connect_to_cpp_server()
        time.sleep(1)

    print("[INFO] ArUco Door Detector Running", flush=True)

    # Set up the ArUco dictionary (4x4 matrix)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()

    try:
        while True:
            header = recvall(sock, 8)
            if not header: break
            width, height = struct.unpack("ii", header)

            frame_size = width * height * 3
            data = recvall(sock, frame_size)
            if not data: break

            # Convert raw RGB bytes from C++ into an OpenCV image
            frame = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 3))
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

            # Scan for the marker
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            msg = ""
            if ids is not None and len(ids) > 0:
                # We found a marker! Get the pixel coordinates of its 4 corners
                c = corners[0][0]
                
                # Convert pixel coordinates to normalized floats (0.0 to 1.0) for the C++ engine
                xmin = min(c[:, 0]) / width
                xmax = max(c[:, 0]) / width
                ymin = min(c[:, 1]) / height
                ymax = max(c[:, 1]) / height
                
                # Send it to C++ mimicking your object detector format!
                msg = f"door 1.00 {xmin:.3f} {ymin:.3f} {xmax:.3f} {ymax:.3f}"
                print(f"[DETECT] Door spotted at {xmin:.2f}, {ymin:.2f}", flush=True)

            if msg:
                payload = msg.encode("utf-8")
                sock.sendall(struct.pack("<I", len(payload)) + payload)
            else:
                sock.sendall(struct.pack("<I", 0))

    except Exception as e:
        print(f"[ERROR] {e}", flush=True)
    finally:
        if sock: sock.close()
        print("[INFO] Door detector stopped", flush=True)

if __name__ == "__main__":
    main()