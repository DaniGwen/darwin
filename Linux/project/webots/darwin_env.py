import gymnasium as gym
from gymnasium import spaces
import numpy as np
import socket
import struct
import time

# This class defines your custom Reinforcement Learning environment
class DarwinOPEnv(gym.Env):
    def __init__(self, server_ip='192.168.1.100', port=1234):
        super(DarwinOPEnv, self).__init__()
        
        # --- Define the Action and Observation Space ---
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(20,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.pi, high=np.pi, shape=(23,), dtype=np.float32)

        # --- Networking Setup ---
        self.server_ip = server_ip
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # We add a timeout to prevent the script from hanging indefinitely
        self.socket.settimeout(10.0) 
        
        self.socket.connect((self.server_ip, self.port))
        print(f"Successfully connected to C++ server at {self.server_ip}:{self.port}")

        # SensorData: 23 doubles, MotorCommands: 20 doubles
        self.sensor_struct = struct.Struct('23d')
        self.command_struct = struct.Struct('20d')

    def _get_obs(self):
        """Receives sensor data from the C++ server."""
        try:
            data = self.socket.recv(self.sensor_struct.size)
            if len(data) < self.sensor_struct.size:
                raise ConnectionError("Connection issue: Received incomplete sensor packet.")
            unpacked_data = self.sensor_struct.unpack(data)
            return np.array(unpacked_data, dtype=np.float32)
        except socket.timeout:
            raise ConnectionError("Connection timed out while waiting for observation.")
        except Exception as e:
            raise ConnectionError(f"An error occurred while receiving data: {e}")


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
        
        # --- FIX: Send the reset command ---
        reset_command = [999.0] * 20
        packed_reset_command = self.command_struct.pack(*reset_command)
        self.socket.sendall(packed_reset_command)
        
        # --- The C++ supervisor will now receive this, reset, step once,
        #     and then enter its main loop where it waits for the NEXT command.
        #     It does NOT automatically send data after reset.
        #     So, we must send a dummy action to get the first observation.
        
        dummy_action = [0.0] * 20
        packed_dummy_action = self.command_struct.pack(*dummy_action)
        self.socket.sendall(packed_dummy_action)

        # --- Now it is safe to wait for the first observation of the new episode ---
        observation = self._get_obs()
        info = {}
        
        return observation, info

    def close(self):
        """Closes the connection."""
        self.socket.close()
        print("Connection closed.")