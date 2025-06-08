import gymnasium as gym
from gymnasium import spaces
import numpy as np
import socket
import struct
import time

class DarwinOPEnv(gym.Env):
    def __init__(self, server_ip='127.0.0.1', port=1234):
        super(DarwinOPEnv, self).__init__()

        # --- MODIFIED: Reduced action and observation space ---
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(18,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.pi, high=np.pi, shape=(24,), dtype=np.float32)

        self.server_ip = server_ip
        self.port = port
        self.socket = None
        self._connect()

        # --- MODIFIED: Updated struct sizes ---
        self.sensor_struct = struct.Struct('24d')
        self.command_struct = struct.Struct('18d')

        self.last_x_position = 0.0
        self.current_x = 0.0

    def _connect(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(25.0)
        print(f"Connecting to Webots C++ server at {self.server_ip}:{self.port}...")
        try:
            self.socket.connect((self.server_ip, self.port))
            print("Connection successful.")
        except socket.error as e:
            print(f"Failed to connect to {self.server_ip}:{self.port}. Error: {e}")
            raise

    def _get_obs(self):
        try:
            packet_size = self.sensor_struct.size
            data = b''
            while len(data) < packet_size:
                chunk = self.socket.recv(packet_size - len(data))
                if not chunk:
                    raise ConnectionError("Connection closed by server while receiving data.")
                data += chunk
            unpacked_data = self.sensor_struct.unpack(data)
            return np.array(unpacked_data, dtype=np.float32)
        except socket.timeout:
            raise ConnectionError("Timeout: No observation received from simulator.")
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

        roll, pitch = observation[18], observation[19]
        self.current_x, height = observation[21], observation[23]

        # --- Enhanced Reward Function ---
        delta_x = self.current_x - self.last_x_position
        progress_reward = 25.0 * max(0.0, delta_x)

        target_height = 0.33
        height_error = abs(height - target_height)
        height_reward = max(0.0, 1.0 - height_error * 10.0)

        balance_penalty = abs(pitch) + abs(roll)
        energy_penalty = 0.005 * np.sum(np.square(action))
        survival_bonus = 1.0

        reward = progress_reward + height_reward + survival_bonus - balance_penalty - energy_penalty

        self.last_x_position = self.current_x
        terminated = height < 0.22 or abs(pitch) > 1.4 or abs(roll) > 1.4

        if terminated:
            reward -= 10.0 + 2.0 * balance_penalty

        return observation, reward, terminated, False, {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        try:
            reset_command = [999.0] * 18
            self.socket.sendall(self.command_struct.pack(*reset_command))
            observation = self._get_obs()
            self.last_x_position = observation[21] # Index for X position is now 21
            self.current_x = observation[21]
            return observation, {}
        except ConnectionError as e:
            print(f"Connection error during reset: {e}. Retrying connection...")
            self._reconnect()
            return self.reset(seed=seed, options=options)

    def _reconnect(self):
        if self.socket:
            self.socket.close()
        time.sleep(2)
        self._connect()

    def close(self):
        print("Closing environment and socket.")
        if self.socket:
            self.socket.close()