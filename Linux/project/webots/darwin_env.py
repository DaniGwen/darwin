import gymnasium as gym
from gymnasium import spaces
import numpy as np
import socket
import struct
import time

class DarwinOPEnv(gym.Env):
    def __init__(self, server_ip='127.0.0.1', port=1234):
        super(DarwinOPEnv, self).__init__()
        
        self.observation_space = spaces.Box(low=-np.pi, high=np.pi, shape=(26,), dtype=np.float32)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(20,), dtype=np.float32)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(20.0) 
        print(f"Connecting to Webots C++ server at {server_ip}:{port}...")
        try:
            self.socket.connect((server_ip, port))
            print("Connection successful.")
        except socket.error as e:
            print(f"Failed to connect to {server_ip}:{port}. Error: {e}")
            raise

        self.sensor_struct = struct.Struct('26d')
        self.command_struct = struct.Struct('20d')
        
        self.last_x_position = 0.0

    def _get_obs(self):
        try:
            data = self.socket.recv(self.sensor_struct.size)
            if not data:
                raise ConnectionError("Connection closed by server.")
            if len(data) < self.sensor_struct.size:
                raise ConnectionError(f"Incomplete packet received. Expected {self.sensor_struct.size}, got {len(data)}.")
            unpacked_data = self.sensor_struct.unpack(data)
            return np.array(unpacked_data, dtype=np.float32)
        except socket.timeout:
            print("Timeout waiting for observation. Trying to reset.")
            raise ConnectionError("Timeout waiting for observation.")
        except Exception as e:
            raise ConnectionError(f"An error occurred receiving data: {e}")

    def step(self, action):
        try:
            scaled_action = action * 2.25
            self.socket.sendall(self.command_struct.pack(*scaled_action))
            observation = self._get_obs()
        except ConnectionError as e:
            print(f"Connection error during step: {e}. Terminating episode.")
            return np.zeros(self.observation_space.shape), -100.0, True, False, {"error": str(e)}
        
        roll, pitch = observation[20], observation[21]
        current_x, current_y, height = observation[23], observation[24], observation[25]
        
        # --- REWARD FUNCTION V7: Stand Tall and Walk! ---
        
        # 1. Primary Goal: Reward forward progress.
        forward_velocity = current_x - self.last_x_position
        progress_reward = 150.0 * forward_velocity

        # 2. Key Incentive: Stand tall!
        target_height = 0.33
        height_reward = 5.0 * np.exp(-100.0 * abs(height - target_height))

        # 3. Stability and Efficiency Penalties
        balance_penalty = 1.0 * (abs(pitch) + abs(roll))
        action_penalty = 0.01 * np.sum(np.square(action))
        
        # Combine all components
        reward = (
            progress_reward +
            height_reward -
            balance_penalty -
            action_penalty
        )
        
        self.last_x_position = current_x
        
        terminated = height < 0.22 or abs(pitch) > 1.4 or abs(roll) > 1.4
        
        if terminated:
            reward = -25.0
        
        return observation, reward, terminated, False, {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        try:
            reset_command = [999.0] * 20
            self.socket.sendall(self.command_struct.pack(*reset_command))
            observation = self._get_obs()
            self.last_x_position = observation[23]
            return observation, {}
        except ConnectionError as e:
            print(f"Connection error during reset: {e}. Retrying connection...")
            self._reconnect()
            return self.reset(seed=seed, options=options)

    def _reconnect(self):
        self.socket.close()
        time.sleep(1)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(20.0)
        try:
            self.socket.connect((self.server_ip, self.port))
            print("Reconnection successful.")
        except socket.error as e:
            print(f"Failed to reconnect: {e}")
            raise

    def close(self):
        print("Closing environment and socket.")
        self.socket.close()