from darwin_env_base import DarwinOPEnvBase
import numpy as np

class DarwinOPBalanceEnv(DarwinOPEnvBase):
    def __init__(self, **kwargs):
        super(DarwinOPBalanceEnv, self).__init__(**kwargs)
        self.target_height = 0.33

    def step(self, action):
        self.episode_steps += 1
        
        observation = self.send_action(action)
        if observation is None:
            # Handle connection error
            return np.zeros(self.observation_space.shape), -10.0, True, False, {}

        roll, pitch = observation[18], observation[19]
        self.current_x, height = observation[21], observation[23]

        # --- Balance-Specific Rewards ---
        height_reward = max(0.0, 1.0 - abs(height - self.target_height) * 10.0)
        pitch_reward = max(0.0, 1.0 - abs(pitch) * 5.0)
        roll_reward = max(0.0, 1.0 - abs(roll) * 5.0)
        balance_reward = (height_reward + pitch_reward + roll_reward) * 2.0

        movement_penalty = abs(self.current_x - self.last_x_position) * 50.0
        energy_penalty = 0.001 * np.sum(np.square(action))
        
        # Encourage two-footed stance
        foot_roll_r = observation[16]
        foot_roll_l = observation[17]
        foot_symmetry_reward = max(0.0, 1.0 - abs(foot_roll_r - foot_roll_l))

        reward = balance_reward + foot_symmetry_reward - movement_penalty - energy_penalty
        
        # Termination conditions
        terminated = (height < 0.22 or abs(pitch) > 1.0 or abs(roll) > 1.0 or self.episode_steps > 1000)
        if terminated:
            reward -= 5.0

        self.last_x_position = self.current_x
        return observation, reward, terminated, False, {}