import gymnasium as gym
from gymnasium import spaces
import numpy as np
import socket
import struct
import time
from typing import Optional

# This base class contains all the common code for connecting to the
# Webots simulator and handling the low-level communication.
class DarwinOPEnvBase(gym.Env):
    def __init__(self, server_ip='127.0.0.1', port=1234):
        super(DarwinOPEnvBase, self).__init__()
        
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(18,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.pi, high=np.pi, shape=(24,), dtype=np.float32)

        self.server_ip = server_ip
        self.port = port
        self.socket = None
        self._connect()

        self.sensor_struct = struct.Struct('24d')
        self.command_struct = struct.Struct('18d')

        # Tracking variables, used by child classes
        self.last_x_position = 0.0
        self.current_x = 0.0
        self.episode_steps = 0

    def _connect(self):
        """Establish connection to Webots controller."""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(30.0)
        print(f"Connecting to Webots C++ server at {self.server_ip}:{self.port}...")
        try:
            self.socket.connect((self.server_ip, self.port))
            print(f"Connection successful on port {self.port}.")
        except socket.error as e:
            print(f"Failed to connect to {self.server_ip}:{self.port}. Error: {e}")
            raise

    def _get_obs(self):
        """Get observation from the simulator."""
        try:
            packet_size = self.sensor_struct.size
            data = b''
            while len(data) < packet_size:
                chunk = self.socket.recv(packet_size - len(data))
                if not chunk:
                    raise ConnectionError("Connection closed by server.")
                data += chunk
            unpacked_data = self.sensor_struct.unpack(data)
            return np.array(unpacked_data, dtype=np.float32)
        except socket.timeout:
            raise ConnectionError(f"Timeout on port {self.port}.")
        except Exception as e:
            raise ConnectionError(f"Error receiving data on port {self.port}: {e}")

    def send_action(self, action):
        """Sends the scaled action to the simulator."""
        try:
            scaled_action = action * 2.25
            self.socket.sendall(self.command_struct.pack(*scaled_action))
            return self._get_obs()
        except ConnectionError as e:
            print(f"Connection error during step on port {self.port}: {e}.")
            return None

    def reset(self, seed=None, options=None):
        """Resets the environment by sending a special command."""
        super().reset(seed=seed)
        self.episode_steps = 0
        try:
            reset_command = [999.0] * 18
            self.socket.sendall(self.command_struct.pack(*reset_command))
            observation = self._get_obs()
            self.last_x_position = observation[21]
            self.current_x = observation[21]
            return observation, {}
        except ConnectionError as e:
            print(f"Connection error during reset on port {self.port}. Retrying connection...")
            self._reconnect()
            return self.reset(seed=seed, options=options)

    def _reconnect(self):
        """Reconnect to Webots if the connection is lost."""
        if self.socket:
            self.socket.close()
        time.sleep(1)
        self._connect()

    def close(self):
        """Clean up the environment."""
        print(f"Closing environment and socket on port {self.port}.")
        if self.socket:
            self.socket.close()

    def step(self, action):
        """This method must be implemented by each child class."""
        raise NotImplementedError