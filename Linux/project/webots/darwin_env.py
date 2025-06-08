import gymnasium as gym
from gymnasium import spaces
import numpy as np
import socket
import struct
import time
from typing import Optional, Dict, Any

class DarwinOPEnv(gym.Env):
    def __init__(self, server_ip='127.0.0.1', port=1234):
        super(DarwinOPEnv, self).__init__()
        
        # Action and observation spaces
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(18,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.pi, high=np.pi, shape=(24,), dtype=np.float32)

        self.server_ip = server_ip
        self.port = port
        self.socket = None
        self._connect()

        # Struct sizes for communication
        self.sensor_struct = struct.Struct('24d')
        self.command_struct = struct.Struct('18d')

        # Tracking variables
        self.last_x_position = 0.0
        self.current_x = 0.0
        self.episode_steps = 0
        self.target_height = 0.33  # Optimal torso height in meters

    def _connect(self):
        """Establish connection to Webots controller"""
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
        """Get observation with added noise for robustness"""
        try:
            packet_size = self.sensor_struct.size
            data = b''
            while len(data) < packet_size:
                chunk = self.socket.recv(packet_size - len(data))
                if not chunk:
                    raise ConnectionError("Connection closed by server while receiving data.")
                data += chunk
            unpacked_data = self.sensor_struct.unpack(data)
            
            # Add small observation noise
            obs = np.array(unpacked_data, dtype=np.float32)
            obs += np.random.normal(0, 0.01, obs.shape)
            return obs
            
        except socket.timeout:
            raise ConnectionError("Timeout: No observation received from simulator.")
        except Exception as e:
            raise ConnectionError(f"An error occurred receiving data: {e}")

    def step(self, action):
        """Execute one time step with enhanced reward function"""
        self.episode_steps += 1
        
        try:
            # Scale action to appropriate range
            scaled_action = action * 2.25
            self.socket.sendall(self.command_struct.pack(*scaled_action))
            observation = self._get_obs()
        except ConnectionError as e:
            print(f"Connection error during step: {e}. Terminating episode.")
            return np.zeros(self.observation_space.shape), -5.0, True, False, {"error": str(e)}

        # Extract key observations
        roll, pitch = observation[18], observation[19]
        self.current_x, height = observation[21], observation[23]
        
        # --- Enhanced Reward Components ---
        # Distance reward (primary driver)
        delta_x = self.current_x - self.last_x_position
        progress_reward = 50.0 * max(0.0, delta_x)
        
        # Height stability reward (wider acceptable range)
        height_error = abs(height - self.target_height)
        height_reward = max(0.0, 1.0 - height_error * 5.0)
        
        # Balance rewards
        pitch_reward = max(0.0, 1.0 - abs(pitch) * 2.0)
        roll_reward = max(0.0, 1.0 - abs(roll) * 2.0)
        
        # Energy penalty (reduced to encourage movement)
        energy_penalty = 0.001 * np.sum(np.square(action))
        
        # Step symmetry reward (encourage alternating gait)
        left_leg_avg = np.mean(observation[0:9])
        right_leg_avg = np.mean(observation[9:18])
        symmetry_reward = max(0.0, 1.0 - abs(left_leg_avg - right_leg_avg))
        
        # Combine rewards
        reward = (progress_reward + 
                 height_reward + 
                 pitch_reward + 
                 roll_reward + 
                 (symmetry_reward * 0.5) - 
                 energy_penalty)
        
        # Termination conditions
        terminated = (height < 0.22 or     # Fallen down
                     abs(pitch) > 1.0 or  # Too much forward/backward tilt
                     abs(roll) > 1.0 or    # Too much side tilt
                     self.episode_steps > 1000)  # Episode timeout
                     
        # Early termination if not making progress
        if self.current_x < 0.1 and self.episode_steps > 50:
            terminated = True
            reward -= 2.0
            
        if terminated:
            # Small penalty for termination
            reward -= 5.0

        self.last_x_position = self.current_x
        return observation, reward, terminated, False, {}

    def reset(self, seed=None, options=None):
        """Reset the environment"""
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
            print(f"Connection error during reset: {e}. Retrying connection...")
            self._reconnect()
            return self.reset(seed=seed, options=options)

    def _reconnect(self):
        """Reconnect to Webots if connection is lost"""
        if self.socket:
            self.socket.close()
        time.sleep(2)
        self._connect()

    def close(self):
        """Clean up environment"""
        print("Closing environment and socket.")
        if self.socket:
            self.socket.close()