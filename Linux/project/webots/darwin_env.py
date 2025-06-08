import gymnasium as gym
from gymnasium import spaces
import numpy as np
import socket
import struct

class DarwinOPEnv(gym.Env):
    def __init__(self, server_ip='127.0.0.1', port=1234):
        super(DarwinOPEnv, self).__init__()
        
        self.observation_space = spaces.Box(low=-np.pi, high=np.pi, shape=(26,), dtype=np.float32)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(20,), dtype=np.float32)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print(f"Connecting to Webots C++ server at {server_ip}:{port}...")
        self.socket.connect((server_ip, port))
        print("Connection successful.")

        self.sensor_struct = struct.Struct('26d')
        self.command_struct = struct.Struct('20d')
        
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
        
        # --- THE RE-TUNED REWARD FUNCTION V5 ---
        
        roll, pitch = observation[20], observation[21]
        current_x, current_y, height = observation[23], observation[24], observation[25]
        
        # 1. Main Goal: Reward forward velocity
        forward_velocity = current_x - self.last_x_position
        forward_reward = 100.0 * forward_velocity

        # 2. Posture Rewards: Guide the agent to stand tall and balanced
        target_height = 0.33
        # Use an exponential function so the reward is high near the target height
        height_reward = 0.5 * np.exp(-50.0 * abs(height - target_height))
        balance_reward = 0.5 * np.exp(-20.0 * (abs(pitch) + abs(roll)))

        # 3. Penalties for inefficient movement
        action_penalty = 0.01 * np.sum(np.square(action))
        sideways_penalty = 5.0 * abs(current_y - self.last_y_position)

        # Combine all components
        reward = (
            forward_reward +
            height_reward +
            balance_reward -
            action_penalty -
            sideways_penalty
        )
        
        # Update state for next step
        self.last_x_position = current_x
        self.last_y_position = current_y
        
        # Termination condition
        terminated = height < 0.24 or abs(pitch) > 1.4 or abs(roll) > 1.4
        if terminated:
            reward = -15.0  # Still a significant penalty for falling

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