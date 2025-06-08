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
        self.server_ip = server_ip
        self.port = port
        self.socket = None
        self._connect()

        self.sensor_struct = struct.Struct('26d')
        self.command_struct = struct.Struct('20d')
        
        # Trackers for calculating rewards and goal achievement
        self.last_x_position = 0.0
        self.current_x = 0.0 # This will be updated each step for the callback

    def _connect(self):
        """Establishes a connection to the server."""
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
        """
        Receives a full data packet from the socket, handling fragmented packets.
        """
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
            raise ConnectionError("Timeout: No observation received from simulator. It may have crashed or is not responding.")
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
        self.current_x, height = observation[23], observation[25] # Update current_x
        
        # --- Reward Function (Unchanged) ---
        progress_reward = 150.0 * (self.current_x - self.last_x_position)
        target_height = 0.33
        height_reward = 5.0 * np.exp(-100.0 * abs(height - target_height))
        balance_penalty = 1.0 * (abs(pitch) + abs(roll))
        action_penalty = 0.01 * np.sum(np.square(action))
        
        reward = progress_reward + height_reward - balance_penalty - action_penalty
        
        self.last_x_position = self.current_x
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
            self.current_x = observation[23]
            return observation, {}
        except ConnectionError as e:
            print(f"Connection error during reset: {e}. Attempting to reconnect...")
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