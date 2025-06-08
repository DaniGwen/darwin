import gymnasium as gym
from gymnasium import spaces
import numpy as np
import socket
import struct


class DarwinOPEnv(gym.Env):
    def __init__(self, server_ip="127.0.0.1", port=1234):
        super(DarwinOPEnv, self).__init__()

        # --- CHANGE 1: Update the observation space to include the 3 new position values ---
        self.observation_space = spaces.Box(
            low=-np.pi, high=np.pi, shape=(26,), dtype=np.float32
        )
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(20,), dtype=np.float32
        )

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print(f"Connecting to Webots C++ server at {server_ip}:{port}...")
        self.socket.connect((server_ip, port))
        print("Connection successful.")

        # --- CHANGE 2: Update the sensor struct to match the new C++ struct ---
        self.sensor_struct = struct.Struct("26d")
        self.command_struct = struct.Struct("20d")

        # This will store the robot's forward position from the previous step
        self.last_x_position = 0

    def _get_obs(self):
        data = self.socket.recv(self.sensor_struct.size)
        if len(data) < self.sensor_struct.size:
            raise ConnectionError("Incomplete packet received.")
        unpacked_data = self.sensor_struct.unpack(data)
        return np.array(unpacked_data, dtype=np.float32)

    def step(self, action):
        # --- 1. Send action to simulation ---
        scaled_action = action * 2.25
        self.socket.sendall(self.command_struct.pack(*scaled_action))

        # --- 2. Receive new state ---
        observation = self._get_obs()

        # --- 3. The New Reward Function ---

        # Unpack observation for clarity. Indices are based on the C++ SensorData struct.
        roll, pitch = observation[20], observation[21]
        current_x, height = observation[23], observation[25]

        # Component 1: Reward for staying upright (high height, low roll/pitch)
        target_height = 0.33  # Target height in meters for standing
        upright_reward = np.exp(-10.0 * (abs(pitch) + abs(roll)))
        height_reward = np.exp(-50.0 * abs(height - target_height))

        # Component 2: Reward for moving forward
        forward_reward = 15.0 * (current_x - self.last_x_position)

        # Component 3: Small penalty for using too much energy (large motor commands)
        action_penalty = 0.005 * np.sum(np.square(action))

        # Component 4: Alive bonus to encourage survival
        alive_bonus = 0.1

        # Combine the rewards
        reward = (
            upright_reward
            + height_reward
            + forward_reward
            - action_penalty
            + alive_bonus
        )

        # --- 4. Update state and check for termination ---
        self.last_x_position = current_x  # Update for the next step's calculation

        # Terminate if the robot falls (height is too low)
        terminated = height < 0.25

        return observation, reward, terminated, False, {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        # Send the special reset command
        reset_command = [999.0] * 20
        self.socket.sendall(self.command_struct.pack(*reset_command))

        # Receive the new initial state after reset
        observation = self._get_obs()

        # Reset the position tracker
        self.last_x_position = observation[23]

        return observation, {}

    def close(self):
        self.socket.close()
