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
        
        # These will track the robot's position between steps
        self.last_x_position = 0
        self.last_y_position = 0

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
        
        # --- THE RE-TUNED REWARD FUNCTION V4 ---
        
        roll, pitch = observation[20], observation[21]
        current_x, current_y, height = observation[23], observation[24], observation[25]
        
        # --- Component 1: Core survival and posture ---
        # Stronger reward for simply being alive and upright
        alive_bonus = 1.0
        balance_reward = 1.5 * np.exp(-15.0 * (abs(pitch) + abs(roll)))
        
        # --- Component 2: Strongly incentivize the desired behavior ---
        target_height = 0.33
        # Reward being tall, penalize being short
        height_reward = 15.0 * (height - target_height)
        
        # Very strong reward for forward velocity to make it the primary goal
        forward_velocity = current_x - self.last_x_position
        forward_reward = 0
        if height > 0.28: # Only give forward reward if standing
            forward_reward = 50.0 * forward_velocity
            
        # --- Component 3: Penalties to refine the movement ---
        sideways_penalty = 10.0 * abs(current_y - self.last_y_position)
        action_penalty = 0.01 * np.sum(np.square(action))
        
        # Combine the rewards
        reward = (
            alive_bonus +
            balance_reward + 
            height_reward +
            forward_reward -
            sideways_penalty -
            action_penalty
        )
        
        # --- 4. Termination ---
        terminated = height < 0.23 or abs(pitch) > 1.4 or abs(roll) > 1.4
        
        if terminated:
            reward = -50.0 # Make falling extremely punishing
        
        # Update state for the next step's calculation
        self.last_x_position = current_x
        self.last_y_position = current_y
        
        return observation, reward, terminated, False, {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        reset_command = [999.0] * 20
        self.socket.sendall(self.command_struct.pack(*reset_command))
        
        observation = self._get_obs()
        
        # Reset position trackers
        self.last_x_position = observation[23]
        self.last_y_position = observation[24]
        
        return observation, {}

    def close(self):
        self.socket.close()

