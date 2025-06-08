import gymnasium as gym
from gymnasium import spaces
import numpy as np
import socket
import struct


class DarwinOPEnv(gym.Env):
    def __init__(self, server_ip="127.0.0.1", port=1234):
        super(DarwinOPEnv, self).__init__()

        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(20,), dtype=np.float32
        )
        self.observation_space = spaces.Box(
            low=-np.pi, high=np.pi, shape=(23,), dtype=np.float32
        )

        self.server_ip = server_ip
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        print(
            f"Attempting to connect to Webots C++ server at {self.server_ip}:{self.port}..."
        )
        self.socket.connect((self.server_ip, self.port))
        print("Connection successful.")

        # Structs for packing/unpacking data
        self.sensor_struct = struct.Struct("23d")
        self.command_struct = struct.Struct("20d")

    def _get_obs(self):
        try:
            data = self.socket.recv(self.sensor_struct.size)
            if len(data) < self.sensor_struct.size:
                raise ConnectionError("Incomplete packet received.")
            return np.array(self.sensor_struct.unpack(data), dtype=np.float32)
        except Exception as e:
            raise ConnectionError(f"Error receiving data: {e}")

    def step(self, action):
        # Scale action from [-1, 1] to motor range and send
        scaled_action = action * 2.25
        self.socket.sendall(self.command_struct.pack(*scaled_action))

        # Receive the new state from the server
        observation = self._get_obs()

        # Calculate reward and termination
        pitch = observation[21]
        reward = np.exp(-5.0 * abs(pitch))
        terminated = abs(pitch) > 1.2

        return observation, reward, terminated, False, {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        # Send the special reset command
        reset_command = [999.0] * 20
        self.socket.sendall(self.command_struct.pack(*reset_command))

        # Receive the new initial state after reset
        observation = self._get_obs()

        return observation, {}

    def close(self):
        self.socket.close()
        print("Connection closed.")
