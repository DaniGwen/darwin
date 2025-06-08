import gymnasium as gym
from gymnasium import spaces
import numpy as np
import socket
import struct

class DarwinOPEnv(gym.Env):
    def __init__(self, server_ip='127.0.0.1', port=1234):
        super(DarwinOPEnv, self).__init__()
        
        # Observation space: 20 joints, 3 IMU, 3 position = 26 values
        self.observation_space = spaces.Box(low=-np.pi, high=np.pi, shape=(26,), dtype=np.float32)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(20,), dtype=np.float32)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(15.0)
        print(f"Connecting to Webots C++ server at {server_ip}:{port}...")
        self.socket.connect((server_ip, port))
        print("Connection successful.")

        self.sensor_struct = struct.Struct('26d')
        self.command_struct = struct.Struct('20d')
        
        self.last_x_position = 0

    def _get_obs(self):
        data = self.socket.recv(self.sensor_struct.size)
        if len(data) < self.sensor_struct.size:
            raise ConnectionError("Incomplete packet received.")
        unpacked_data = self.sensor_struct.unpack(data)
        return np.array(unpacked_data, dtype=np.float32)

    def step(self, action):
        scaled_action = action * 2.25
        self.socket.sendall(self.command_struct.pack(*scaled_action))
        
        observation = self._get_obs()
        
        # --- THE SIMPLIFIED REWARD FUNCTION V6 ---
        
        roll, pitch = observation[20], observation[21]
        
        # 1. Main Goal: Survive! Give a large, constant reward for every step.
        alive_bonus = 1.0

        # 2. Secondary Goal: Stay upright. Give a small bonus for good balance.
        balance_reward = 0.1 * np.exp(-10.0 * (abs(pitch) + abs(roll)))
        
        # Combine the rewards
        reward = alive_bonus + balance_reward
        
        # Termination condition remains the same
        terminated = abs(pitch) > 1.4 or abs(roll) > 1.4
        
        # If it falls, it no longer gets the alive_bonus, which is the main penalty.
        
        return observation, reward, terminated, False, {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        reset_command = [999.0] * 20
        self.socket.sendall(self.command_struct.pack(*reset_command))
        
        observation = self._get_obs()
        
        # We no longer need to track position for this simplified reward
        
        return observation, {}

    def close(self):
        self.socket.close()
