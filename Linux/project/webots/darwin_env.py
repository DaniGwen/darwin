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
        self.socket.settimeout(15.0) 
        print(f"Connecting to Webots C++ server at {server_ip}:{port}...")
        self.socket.connect((server_ip, port))
        print("Connection successful.")

        self.sensor_struct = struct.Struct('26d')
        self.command_struct = struct.Struct('20d')
        
        # Trackers for calculating rewards
        self.last_x_position = 0.0
        self.last_y_position = 0.0

    def _get_obs(self):
        try:
            data = self.socket.recv(self.sensor_struct.size)
            if len(data) < self.sensor_struct.size:
                raise ConnectionError("Incomplete packet received.")
            unpacked_data = self.sensor_struct.unpack(data)
            return np.array(unpacked_data, dtype=np.float32)
        except socket.timeout:
            raise ConnectionError("Connection timed out while waiting for observation.")
        except Exception as e:
            raise ConnectionError(f"An error occurred receiving data: {e}")

    def step(self, action):
        scaled_action = action * 2.25
        self.socket.sendall(self.command_struct.pack(*scaled_action))
        
        observation = self._get_obs()
        
        # --- REWARD FUNCTION V6: Progressive & Stable ---
        
        roll, pitch, yaw = observation[20], observation[21], observation[22]
        current_x, current_y, height = observation[23], observation[24], observation[25]
        
        # 1. Primary Goal: Alive Bonus (constant reward for not falling)
        alive_bonus = 1.0

        # 2. Main Incentive: Progress towards the positive X direction
        # This rewards the agent for the change in position, not just the raw speed.
        progress_reward = 100 * (current_x - self.last_x_position)
        
        # 3. Posture Control: Keep the robot upright
        # Small penalty for tilting helps maintain balance.
        balance_penalty = 0.5 * (abs(pitch) + abs(roll))
        
        # 4. Efficiency Control: Penalize jerky movements
        # This encourages smoother, more energy-efficient actions.
        action_penalty = 0.01 * np.sum(np.square(action))

        # Combine the rewards
        reward = (
            alive_bonus + 
            progress_reward - 
            balance_penalty -
            action_penalty
        )
        
        # Update state for the next step's calculation
        self.last_x_position = current_x
        self.last_y_position = current_y
        
        # Termination condition: has the robot fallen?
        terminated = height < 0.22 or abs(pitch) > 1.4 or abs(roll) > 1.4
        
        if terminated:
            reward = -20.0  # A large penalty for falling to discourage it.
        
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