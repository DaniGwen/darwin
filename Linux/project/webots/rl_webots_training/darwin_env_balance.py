from darwin_env_base import DarwinOPEnvBase
import numpy as np

class DarwinOPBalanceEnv(DarwinOPEnvBase):
    """
    Version 3 of the balancing environment, specifically designed to prevent the 
    agent from learning to "sit on its knees".
    
    Key optimizations include:
    1.  A multiplicative reward structure that heavily punishes incorrect height.
    2.  A stricter height termination condition to reduce the room for exploits.
    3.  A reward for keeping knees straight to encourage a tall posture.
    """
    def __init__(self, **kwargs):
        super(DarwinOPBalanceEnv, self).__init__(**kwargs)
        self.target_height = 0.33
        self.last_action = np.zeros(self.action_space.shape, dtype=np.float32)

    def send_action(self, action):
        """Sends the scaled action to the simulator."""
        try:
            scaled_action = action * 0.4 
            self.socket.sendall(self.command_struct.pack(*scaled_action))
            return self._get_obs()
        except ConnectionError as e:
            print(f"Connection error during step on port {self.port}: {e}.")
            return None

    def step(self, action):
        self.episode_steps += 1
        
        observation = self.send_action(action)
        if observation is None:
            return np.zeros(self.observation_space.shape), -15.0, True, False, {}

        roll, pitch = observation[18], observation[19]
        self.current_x, height = observation[21], observation[23]

        # --- OPTIMIZATION 1: Heavily Penalize Incorrect Height ---
        # The height reward now acts as a dominant multiplier. If the height is
        # wrong, the total reward is crushed, regardless of how stable pitch/roll are.
        height_reward = np.exp(-35.0 * abs(height - self.target_height))
        pitch_reward = np.exp(-5.0 * abs(pitch))
        roll_reward = np.exp(-5.0 * abs(roll))
        
        # The total balance reward is now multiplicative.
        balance_reward = 8.0 * height_reward * (pitch_reward + roll_reward)

        # --- OPTIMIZATION 2: Encourage Straight Knees ---
        # This reward explicitly encourages the robot to stand up straight.
        knee_r, knee_l = observation[12], observation[13]
        upright_knee_reward = 1.0 * np.exp(-20.0 * (abs(knee_r) + abs(knee_l)))

        # --- Penalties (largely unchanged) ---
        movement_penalty = 5.0 * abs(self.current_x - self.last_x_position)
        energy_penalty = 0.001 * np.sum(np.square(action))
        action_smoothness_penalty = 0.1 * np.sum(np.square(action - self.last_action))

        # Combine all reward and penalty terms
        reward = (balance_reward + 
                  upright_knee_reward -
                  movement_penalty - 
                  energy_penalty - 
                  action_smoothness_penalty)
        
        self.last_action = action
        self.last_x_position = self.current_x

        # --- OPTIMIZATION 3: Stricter Termination ---
        # The height threshold is raised from 0.22 to 0.28. The robot now has a
        # much smaller margin to "cheat" by kneeling without terminating the episode.
        terminated = (height < 0.28 or abs(pitch) > 1.0 or abs(roll) > 1.0 or self.episode_steps > 1000)
        if terminated:
            reward = -15.0 # Increased penalty for failure

        return observation, reward, terminated, False, {}

    def reset(self, seed=None, options=None):
        self.last_action = np.zeros(self.action_space.shape, dtype=np.float32)
        return super().reset(seed=seed, options=options)