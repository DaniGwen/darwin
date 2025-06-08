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
        # 1. Send action to simulation
        scaled_action = action * 2.25
        self.socket.sendall(self.command_struct.pack(*scaled_action))
        
        # 2. Receive new state
        observation = self._get_obs()
        
        # --- 3. THE ADVANCED REWARD FUNCTION ---
        
        roll, pitch = observation[20], observation[21]
        current_x, height = observation[23], observation[25]
        
        # --- Component 1: Reward for being upright and tall ---
        target_height = 0.33
        # Stronger penalty for being too low, less penalty for being a bit too high
        height_penalty = 100.0 * max(0, target_height - height) 
        # Reward for being balanced
        balance_reward = np.exp(-15.0 * (abs(pitch) + abs(roll)))
        
        # --- Component 2: Reward for efficient forward motion ---
        forward_progress = current_x - self.last_x_position
        forward_reward = 0
        # CRITICAL: Only give forward reward if the robot is actually standing up!
        if height > 0.28:  # 28cm is a reasonable "standing" threshold
            forward_reward = 25.0 * forward_progress
        
        # --- Component 3: Penalties to shape behavior ---
        # Penalty for using too much energy (discourages wild movements)
        action_penalty = 0.01 * np.sum(np.square(action))
        # Penalty for falling over (large negative reward)
        fall_penalty = -5.0 if abs(pitch) > 1.4 or abs(roll) > 1.4 else 0
        
        # Combine the rewards and penalties
        reward = (
            balance_reward + 
            forward_reward - 
            height_penalty - 
            action_penalty + 
            fall_penalty
        )
        
        # 4. Update state and check for termination
        self.last_x_position = current_x
        terminated = abs(pitch) > 1.4 or abs(roll) > 1.4
        
        return observation, reward, terminated, False, {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        reset_command = [999.0] * 20
        self.socket.sendall(self.command_struct.pack(*reset_command))
        
        observation = self._get_obs()
        
        self.last_x_position = observation[23]
        
        return observation, {}

    def close(self):
        self.socket.close()