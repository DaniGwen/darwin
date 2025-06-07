import gymnasium as gym
from gymnasium import spaces
import numpy as np
import socket
import struct
import time


# This class defines your custom Reinforcement Learning environment
class DarwinOPEnv(gym.Env):
    def __init__(self, server_ip="192.168.1.100", port=1234):
        super(DarwinOPEnv, self).__init__()

        # --- Define the Action and Observation Space ---
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(20,), dtype=np.float32
        )
        self.observation_space = spaces.Box(
            low=-np.pi, high=np.pi, shape=(23,), dtype=np.float32
        )

        # --- Networking Setup ---
        self.server_ip = server_ip
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.server_ip, self.port))
        print(f"Successfully connected to C++ server at {self.server_ip}:{self.port}")

        # SensorData: 23 doubles, MotorCommands: 20 doubles
        # We now need separate structs for sending and receiving
        self.sensor_struct = struct.Struct("23d")
        self.command_struct = struct.Struct("20d")

    def _get_obs(self):
        """Receives sensor data from the C++ server."""
        data = self.socket.recv(self.sensor_struct.size)
        if not data:
            raise ConnectionError("Connection to C++ server lost.")
        unpacked_data = self.sensor_struct.unpack(data)
        return np.array(unpacked_data, dtype=np.float32)

    def step(self, action):
        """Sends an action to the environment and gets the next state and reward."""
        # Scale the agent's action from [-1, 1] to the robot's motor range.
        scaled_action = action * 2.25

        # Pack and send the action
        packed_action = self.command_struct.pack(*scaled_action)
        self.socket.sendall(packed_action)

        # Get the new state from the simulation
        observation = self._get_obs()

        # Calculate the reward
        pitch = observation[21]
        reward = np.exp(-5.0 * abs(pitch))

        # Check if the episode is terminated (robot fell)
        terminated = abs(pitch) > 1.2
        truncated = False
        info = {}

        return observation, reward, terminated, truncated, info

    def reset(self, seed=None, options=None):
        """
        Resets the environment for a new episode.
        This is the corrected logic.
        """
        super().reset(seed=seed)

        print("Resetting environment...")

        # --- FIX: Send the reset command FIRST to break the deadlock ---
        # Create a command with the magic number (999.0) in the first slot.
        reset_command = [999.0] * 20

        # Pack and send the reset command
        packed_reset_command = self.command_struct.pack(*reset_command)
        self.socket.sendall(packed_reset_command)

        # The C++ supervisor will now receive this, reset the simulation,
        # and then send back the new initial sensor state.

        # --- Now it is safe to wait for the observation ---
        observation = self._get_obs()
        info = {}

        return observation, info

    def close(self):
        """Closes the connection."""
        self.socket.close()
        print("Connection closed.")
