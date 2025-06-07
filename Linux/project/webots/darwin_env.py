import gymnasium as gym
from gymnasium import spaces
import numpy as np
import socket
import struct


# This class defines your custom Reinforcement Learning environment
class DarwinOPEnv(gym.Env):
    def __init__(self, server_ip="192.168.1.100", port=1234):
        super(DarwinOPEnv, self).__init__()

        # --- Define the Action and Observation Space ---
        # These must match the data from your robot.

        # Action space: 20 motors, values from -1 to 1 (we will scale this later)
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(20,), dtype=np.float32
        )

        # Observation space: 20 joints + 3 IMU values
        # The range should be set to the physical limits of your sensors.
        self.observation_space = spaces.Box(
            low=-np.pi, high=np.pi, shape=(23,), dtype=np.float32
        )

        # --- Networking Setup ---
        self.server_ip = server_ip
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.server_ip, self.port))
        print(f"Successfully connected to C++ server at {self.server_ip}:{self.port}")

        # The C++ server expects data in specific struct formats.
        # SensorData: 23 doubles
        # MotorCommands: 20 doubles
        self.sensor_struct = struct.Struct("23d")
        self.command_struct = struct.Struct("20d")

    def _get_obs(self):
        """Receives sensor data from the C++ server."""
        data = self.socket.recv(self.sensor_struct.size)
        if not data:
            raise ConnectionError("Connection to C++ server lost.")

        # Unpack the received bytes into a tuple of 23 doubles
        unpacked_data = self.sensor_struct.unpack(data)
        return np.array(unpacked_data, dtype=np.float32)

    def step(self, action):
        """
        This is the core of the RL loop. It does three things:
        1. Takes an action from the agent.
        2. Sends it to the simulation.
        3. Returns the new observation, reward, and other info.
        """

        # --- 1. Scale and Send Action ---
        # The RL agent outputs actions in a normalized [-1, 1] range.
        # We need to scale them to the robot's actual motor range (e.g., -2.25 to 2.25 radians).
        # Note: You can have different scaling for each joint.
        scaled_action = action * 2.25

        # Pack the action into the struct format and send it
        packed_action = self.command_struct.pack(*scaled_action)
        self.socket.sendall(packed_action)

        # --- 2. Get New State from Simulation ---
        observation = self._get_obs()

        # --- 3. Calculate the Reward ---
        # This is where you define the goal of the task.
        # Example: Reward for staying upright.
        # The observation array has [joint_pos_0, ..., joint_pos_19, roll, pitch, yaw]
        pitch = observation[21]

        # Reward is high when pitch is close to 0, decreases as it tilts
        reward = np.exp(-5.0 * abs(pitch))

        # --- 4. Check if the episode is over (Done condition) ---
        # Example: The episode ends if the robot falls over (pitch > ~1.2 radians or 70 degrees).
        terminated = abs(pitch) > 1.2

        # Truncated is for episodes that end due to a time limit, not failure.
        truncated = False

        # info dictionary can be used for debugging
        info = {}

        return observation, reward, terminated, truncated, info

    def reset(self, seed=None, options=None):
        """
        Called at the beginning of each new training episode.
        It should reset the simulation to a starting state.
        """
        super().reset(seed=seed)

        # --- Signal the C++ Server to Reset ---
        # We can send a special "reset" command. A simple way is to send an action
        # with a specific magic number that the C++ server recognizes.
        # Here, we'll just rely on the C++ server to handle the first observation request
        # after an episode ends as a cue to reset. For a more robust system, you'd
        # have a dedicated reset signal.

        print("Resetting environment...")

        # For this simple setup, we assume the C++ server resets itself
        # when the simulation is reset in Webots. We get the first observation.
        observation = self._get_obs()
        info = {}

        return observation, info

    def close(self):
        """Closes the connection."""
        self.socket.close()
        print("Connection closed.")
