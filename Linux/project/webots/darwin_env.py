import gymnasium as gym
from gymnasium import spaces
import numpy as np
import socket
import struct

class DarwinOPEnv(gym.Env):
    def __init__(self, server_ip='192.168.1.100', port=1234):
        super(DarwinOPEnv, self).__init__()
        
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(20,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.pi, high=np.pi, shape=(23,), dtype=np.float32)

        self.server_ip = server_ip
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(15.0) 
        
        self.socket.connect((self.server_ip, self.port))
        print(f"Connected to C++ server at {self.server_ip}:{self.port}")

        self.sensor_struct = struct.Struct('23d')
        self.command_struct = struct.Struct('20d')

        # --- NEW: Perform Handshake on Initialization ---
        print("Sending START signal...")
        self.socket.sendall(b'S') # Send the 'S' character as bytes
        
        print("Waiting for initial observation...")
        # This first observation is received here and stored for the first reset() call
        self._initial_observation = self._get_obs()
        print("Handshake complete. Initial observation received.")

    def _get_obs(self):
        try:
            data = self.socket.recv(self.sensor_struct.size)
            if len(data) < self.sensor_struct.size:
                raise ConnectionError("Incomplete sensor packet received.")
            return np.array(self.sensor_struct.unpack(data), dtype=np.float32)
        except socket.timeout:
            raise ConnectionError("Connection timed out while waiting for observation.")
        except Exception as e:
            raise ConnectionError(f"An error occurred receiving data: {e}")

    def step(self, action):
        scaled_action = action * 2.25
        packed_action = self.command_struct.pack(*scaled_action)
        self.socket.sendall(packed_action)
        observation = self._get_obs()
        pitch = observation[21]
        reward = np.exp(-5.0 * abs(pitch))
        terminated = abs(pitch) > 1.2
        return observation, reward, terminated, False, {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        # --- NEW: Simplified Reset Logic ---
        # The very first time reset is called, we return the observation from the handshake.
        if self._initial_observation is not None:
            obs_to_return = self._initial_observation
            self._initial_observation = None  # Consume it so it's only used once
            return obs_to_return, {}

        # For all subsequent resets, send the reset command and wait for the response.
        print("Resetting simulation...")
        reset_command = [999.0] * 20
        packed_reset_command = self.command_struct.pack(*reset_command)
        self.socket.sendall(packed_reset_command)
        observation = self._get_obs()
        return observation, {}

    def close(self):
        self.socket.close()
        print("Connection closed.")