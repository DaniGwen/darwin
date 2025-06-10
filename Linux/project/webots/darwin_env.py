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
        
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(18,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.pi, high=np.pi, shape=(24,), dtype=np.float32)

        self.server_ip = server_ip
        self.port = port
        self.socket = None
        self._connect()

        self.sensor_struct = struct.Struct('24d')
        self.command_struct = struct.Struct('18d')

        self.learning_phase = 'balance'
        self.balance_success_duration = 700
        self.steps_balanced_continuously = 0

        self.last_x_position = 0.0
        self.current_x = 0.0
        self.episode_steps = 0
        self.target_height = 0.33

    def _connect(self):
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
        try:
            packet_size = self.sensor_struct.size
            data = b''
            while len(data) < packet_size:
                chunk = self.socket.recv(packet_size - len(data))
                if not chunk:
                    raise ConnectionError("Connection closed by server while receiving data.")
                data += chunk
            unpacked_data = self.sensor_struct.unpack(data)
            
            obs = np.array(unpacked_data, dtype=np.float32)
            return obs
            
        except socket.timeout:
            raise ConnectionError(f"Timeout: No observation received from simulator on port {self.port}.")
        except Exception as e:
            raise ConnectionError(f"An error occurred receiving data on port {self.port}: {e}")

    def step(self, action):
        self.episode_steps += 1
        
        try:
            scaled_action = action * 2.25
            self.socket.sendall(self.command_struct.pack(*scaled_action))
            observation = self._get_obs()
        except ConnectionError as e:
            print(f"Connection error during step on port {self.port}: {e}. Terminating episode.")
            return np.zeros(self.observation_space.shape), -10.0, True, False, {"error": str(e)}

        roll, pitch = observation[18], observation[19]
        self.current_x, height = observation[21], observation[23]
        
        # --- Persistent Balance Rewards ---
        height_reward = max(0.0, 1.0 - abs(height - self.target_height) * 10.0)
        pitch_reward = max(0.0, 1.0 - abs(pitch) * 5.0)
        roll_reward = max(0.0, 1.0 - abs(roll) * 5.0)
        balance_reward = (height_reward + pitch_reward + roll_reward) * 2.0
        
        energy_penalty = 0.001 * np.sum(np.square(action))

        # --- Arm Posture "Soft Limits" Penalty ---
        arm_posture_penalty = 0.0
        arm_joint_indices = [0, 1, 2, 3, 4, 5]
        natural_range = 0.5
        for i in arm_joint_indices:
            if abs(action[i]) > natural_range:
                arm_posture_penalty += (abs(action[i]) - natural_range)**2
        arm_posture_penalty *= 0.5

        # --- MODIFICATION: Foot Symmetry Reward (Applied in BOTH phases) ---
        # This encourages keeping both feet planted or in a similar phase of a gait.
        foot_roll_r = observation[16]
        foot_roll_l = observation[17]
        foot_symmetry_reward = max(0.0, 1.0 - abs(foot_roll_r - foot_roll_l))

        # --- Reward Calculation based on Learning Phase ---
        if self.learning_phase == 'balance':
            # --- Phase 1: Balance Reward ---
            movement_penalty = abs(self.current_x - self.last_x_position) * 50.0
            
            # The goal is now to balance while keeping arms in a natural pose AND feet symmetrical.
            reward = (balance_reward + 
                      foot_symmetry_reward - # Encourage two-footed balance from the start
                      movement_penalty - 
                      energy_penalty - 
                      arm_posture_penalty)

            is_balanced_now = (height > 0.30 and abs(pitch) < 0.25 and abs(roll) < 0.25)
            if is_balanced_now:
                self.steps_balanced_continuously += 1
            else:
                self.steps_balanced_continuously = 0

            if self.steps_balanced_continuously >= self.balance_success_duration:
                self.learning_phase = 'walk'
                self.steps_balanced_continuously = 0
                print(f"\n--- GOAL MET (Env on port {self.port}): BALANCING MASTERED! TRANSITIONING TO WALK PHASE. ---\n")
                reward += 100.0

        else: # self.learning_phase == 'walk'
            # --- Phase 2: Walk Reward ---
            delta_x = self.current_x - self.last_x_position
            progress_reward = 50.0 * max(0.0, delta_x)

            # The final reward encourages progress WHILE maintaining balance and good posture.
            reward = (progress_reward + 
                      balance_reward + 
                      foot_symmetry_reward - # Continue rewarding foot symmetry
                      arm_posture_penalty - 
                      energy_penalty)
        
        # --- Termination Conditions ---
        terminated = (height < 0.22 or abs(pitch) > 1.0 or abs(roll) > 1.0 or self.episode_steps > 1000)
                     
        if self.learning_phase == 'walk' and self.current_x < 0.1 and self.episode_steps > 150:
            terminated = True
            reward -= 2.0
            
        if terminated:
            reward -= 5.0

        self.last_x_position = self.current_x
        return observation, reward, terminated, False, {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.episode_steps = 0
        self.steps_balanced_continuously = 0
        
        try:
            reset_command = [999.0] * 18
            self.socket.sendall(self.command_struct.pack(*reset_command))
            observation = self._get_obs()
            self.last_x_position = observation[21]
            self.current_x = observation[21]
            return observation, {}
        except ConnectionError as e:
            print(f"Connection error during reset on port {self.port}: {e}. Retrying connection...")
            self._reconnect()
            return self.reset(seed=seed, options=options)

    def _reconnect(self):
        if self.socket:
            self.socket.close()
        time.sleep(1)
        self._connect()

    def close(self):
        print(f"Closing environment and socket on port {self.port}.")
        if self.socket:
            self.socket.close()