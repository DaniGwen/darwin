from darwin_env_base import DarwinOPEnvBase
import numpy as np

class DarwinOPBalanceEnv(DarwinOPEnvBase):
    def __init__(self, **kwargs):
        super(DarwinOPBalanceEnv, self).__init__(**kwargs)
        self.target_height = 0.33
        # Initialize a variable to store the last action taken
        self.last_action = np.zeros(self.action_space.shape, dtype=np.float32)

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

        # --- Penalties ---
        movement_penalty = abs(self.current_x - self.last_x_position) * 50.0
        energy_penalty = 0.001 * np.sum(np.square(action))
        
        # Encourage two-footed stance (keeps feet parallel)
        foot_roll_r = observation[16]
        foot_roll_l = observation[17]
        foot_symmetry_reward = max(0.0, 1.0 - abs(foot_roll_r - foot_roll_l))

        # Stance Width Penalty
        hip_roll_r = observation[8]
        hip_roll_l = observation[9]
        stance_width_penalty = 0.5 * (abs(hip_roll_r) + abs(hip_roll_l))

        # Shoulder Posture Penalty
        shoulder_pitch_r = observation[0]
        shoulder_pitch_l = observation[1]
        shoulder_penalty = 0.0
        if shoulder_pitch_r > 0.5:
            shoulder_penalty += (shoulder_pitch_r - 0.5)**2
        if shoulder_pitch_l > 0.5:
            shoulder_penalty += (shoulder_pitch_l - 0.5)**2
        shoulder_penalty *= 1.5

        # Action Smoothness Penalty
        action_smoothness_penalty = 0.1 * np.sum(np.square(action - self.last_action))

        # --- MODIFICATION: Ankle Posture Penalty ---
        # This penalizes the agent for tilting its ankles too much,
        # encouraging flat feet for a stable base.
        # Ankle Pitch (R:14, L:15), Ankle Roll (R:16, L:17)
        ankle_pitch_r = observation[14]
        ankle_pitch_l = observation[15]
        # The penalty is the sum of the absolute values of the angles.
        ankle_penalty = 0.5 * (abs(ankle_pitch_r) + abs(ankle_pitch_l) + abs(foot_roll_r) + abs(foot_roll_l))


        reward = (balance_reward + 
                  foot_symmetry_reward - 
                  movement_penalty - 
                  energy_penalty -
                  stance_width_penalty -
                  shoulder_penalty -
                  action_smoothness_penalty -
                  ankle_penalty) # Subtract the new ankle penalty
        
        # --- Update last action for the next step ---
        self.last_action = action

        # Termination conditions
        terminated = (height < 0.22 or abs(pitch) > 1.0 or abs(roll) > 1.0 or self.episode_steps > 1000)
        if terminated:
            reward -= 5.0

        self.last_x_position = self.current_x
        return observation, reward, terminated, False, {}

    def reset(self, seed=None, options=None):
        self.last_action = np.zeros(self.action_space.shape, dtype=np.float32)
        return super().reset(seed=seed, options=options)
