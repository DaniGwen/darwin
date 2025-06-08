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
        
        # --- THE RE-TUNED REWARD FUNCTION V3 ---
        
        roll, pitch = observation[20], observation[21]
        current_x, current_y, height = observation[23], observation[24], observation[25]
        
        # --- Component 1: Staying Upright ---
        target_height = 0.33
        # Reduced penalty for not being at the perfect height
        height_reward = 1.5 * height 
        # Strong reward for being balanced
        balance_reward = 0.5 * np.exp(-20.0 * (abs(pitch) + abs(roll)))
        
        # --- Component 2: Forward Motion ---
        forward_velocity = current_x - self.last_x_position
        # Stronger reward for moving forward
        forward_reward = 0
        if height > 0.28: # Only reward forward motion if standing
            forward_reward = 25.0 * forward_velocity
            
        # --- Component 3: Penalties ---
        # Penalty for sideways motion to encourage straight walking
        sideways_penalty = 10.0 * abs(current_y - self.last_y_position)
        # Penalty for using too much energy
        action_penalty = 0.005 * np.sum(np.square(action))
        # Increased alive bonus to make survival more valuable
        alive_bonus = 0.5
        
        # Combine the rewards
        reward = (
            height_reward +
            balance_reward + 
            forward_reward +
            alive_bonus -
            sideways_penalty -
            action_penalty
        )
        
        # Update state for the next step's calculation
        self.last_x_position = current_x
        self.last_y_position = current_y
        
        # Terminate if the robot falls
        terminated = height < 0.25 # Back to a simple height check, but our rewards now punish low height
        
        if terminated:
            reward = -20.0 # Large penalty for falling
        
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

