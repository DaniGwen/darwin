from darwin_env_base import DarwinOPEnvBase
import numpy as np

class DarwinOPWalkEnv(DarwinOPEnvBase):
    def __init__(self, **kwargs):
        super(DarwinOPWalkEnv, self).__init__(**kwargs)
        self.target_height = 0.33

    def step(self, action):
        self.episode_steps += 1

        observation = self.send_action(action)
        if observation is None:
            return np.zeros(self.observation_space.shape), -10.0, True, False, {}

        roll, pitch = observation[18], observation[19]
        self.current_x, height = observation[21], observation[23]

        # --- Walk-Specific Rewards ---
        # Reward forward progress
        delta_x = self.current_x - self.last_x_position
        progress_reward = 50.0 * max(0.0, delta_x)

        # Reward for maintaining balance while walking
        height_reward = max(0.0, 1.0 - abs(height - self.target_height) * 10.0)
        pitch_reward = max(0.0, 1.0 - abs(pitch) * 5.0)
        roll_reward = max(0.0, 1.0 - abs(roll) * 5.0)
        balance_reward = (height_reward + pitch_reward + roll_reward) * 2.0

        # Penalize unnatural arm postures
        arm_posture_penalty = 0.0
        arm_joint_indices = [0, 1, 2, 3, 4, 5]
        natural_range = 0.5
        for i in arm_joint_indices:
            if abs(action[i]) > natural_range:
                arm_posture_penalty += (abs(action[i]) - natural_range)**2
        arm_posture_penalty *= 0.5
        
        energy_penalty = 0.001 * np.sum(np.square(action))

        reward = progress_reward + balance_reward - arm_posture_penalty - energy_penalty
        
        terminated = (height < 0.22 or abs(pitch) > 1.0 or abs(roll) > 1.0 or self.episode_steps > 1000)
        if terminated:
            reward -= 5.0

        self.last_x_position = self.current_x
        return observation, reward, terminated, False, {}