import gymnasium as gym
from gymnasium import spaces
import numpy as np
import socket
import struct
import time

# --- Constants for vision-based environment ---
CAMERA_WIDTH = 160
CAMERA_HEIGHT = 120
IMAGE_SHAPE = (CAMERA_HEIGHT, CAMERA_WIDTH, 3) # We'll use RGB, not BGRA
NUM_SENSORS = 24
IMAGE_SIZE_BGRA = CAMERA_WIDTH * CAMERA_HEIGHT * 4

# --- Base class for vision environment ---
# This handles the complex data packet from the new supervisor
class DarwinOPVisionBase(gym.Env):
    def __init__(self, server_ip='127.0.0.1', port=1234):
        super().__init__()
        self.server_ip = server_ip
        self.port = port
        self.socket = None
        self._connect()
        # Combined data packet struct
        self.vision_sensor_struct = struct.Struct(f'24d {IMAGE_SIZE_BGRA}B')
        self.command_struct = struct.Struct('20d') # Now controls 20 motors

    def _connect(self): # (Same as before)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(30.0)
        try:
            self.socket.connect((self.server_ip, self.port))
            print(f"Vision Env connected on port {self.port}.")
        except socket.error as e: raise e

    def get_full_observation(self):
        """Receives and unpacks the combined image and sensor data packet."""
        try:
            data = self.socket.recv(self.vision_sensor_struct.size, socket.MSG_WAITALL)
            if not data: raise ConnectionError("Connection closed.")
            
            unpacked_data = self.vision_sensor_struct.unpack(data)
            
            sensors = np.array(unpacked_data[:NUM_SENSORS], dtype=np.float32)
            image_bgra = np.array(unpacked_data[NUM_SENSORS:], dtype=np.uint8).reshape((CAMERA_HEIGHT, CAMERA_WIDTH, 4))
            # Convert BGRA to RGB for standard processing
            image_rgb = image_bgra[:, :, [2, 1, 0]]
            
            return sensors, image_rgb
        except Exception as e:
            print(f"Error getting full observation: {e}")
            return None, None
            
    def send_full_action(self, action_vec_20):
        """Sends the full 20-motor command vector."""
        self.socket.sendall(self.command_struct.pack(*action_vec_20))
        
    def close(self):
        if self.socket: self.socket.close()
    
    def step(self, action): raise NotImplementedError
    def reset(self, seed=None, options=None): raise NotImplementedError

# --- Specialized Environment for Grasping ---
class DarwinOPGraspEnv(DarwinOPVisionBase):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        # ACTION: Controls only the right arm (ShoulderR, ArmUpperR, ArmLowerR) and hand (Wrist, Gripper)
        # Note: We are simplifying to 3 arm joints for the initial training.
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)
        
        # OBSERVATION: The camera image
        self.observation_space = spaces.Box(low=0, high=255, shape=IMAGE_SHAPE, dtype=np.uint8)

        self.full_action_vector = np.zeros(20) # To hold commands for all motors

    def reset(self, seed=None, options=None):
        # On reset, tell the supervisor to reset the simulation
        self.send_full_action([999.0] * 20)
        _, image = self.get_full_observation()
        # You would also add logic here to randomize the target object's position
        return image, {}

    def step(self, action):
        # The trained model only outputs 3 actions for the arm
        arm_action = action
        
        # We keep all other motors at their current position (or a neutral one)
        # This assumes the 'balance' model would be running in the final C++ agent.
        # For training, we can just keep them still.
        # Right Arm motor indices: ShoulderR(0), ArmUpperR(2), ArmLowerR(4)
        self.full_action_vector[0] = arm_action[0]
        self.full_action_vector[2] = arm_action[1]
        self.full_action_vector[4] = arm_action[2]
        
        self.send_full_action(self.full_action_vector)
        sensors, image = self.get_full_observation()

        # --- Grasping Reward Function (Example) ---
        # A real implementation requires finding the object in the image or getting its position from the simulator.
        # For now, let's pretend we have the object's position.
        gripper_pos = sensors[[0,2,4]] # Placeholder
        object_pos = np.array([0.5, 0.5, 0.5]) # Placeholder
        
        distance_to_object = np.linalg.norm(gripper_pos - object_pos)
        
        # Reward is higher the closer the gripper is to the object
        reward = -distance_to_object
        
        terminated = self.episode_steps > 200
        if distance_to_object < 0.05:
            reward += 100 # Huge bonus for reaching the object
            terminated = True
            
        return image, reward, terminated, False, {}

