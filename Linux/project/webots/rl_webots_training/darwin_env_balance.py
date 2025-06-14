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

        # Stance Width Penalty (Hip Roll)
        hip_roll_r = observation[8]
        hip_roll_l = observation[9]
        stance_width_penalty = 0.5 * (abs(hip_roll_r) + abs(hip_roll_l))

        # Symmetrical Shoulder Posture Penalty
        shoulder_pitch_r = observation[0]
        shoulder_pitch_l = observation[1]
        shoulder_penalty = 0.0
        limit = 0.2
        if abs(shoulder_pitch_r) > limit:
            shoulder_penalty += (abs(shoulder_pitch_r) - limit)**2
        if abs(shoulder_pitch_l) > limit:
            shoulder_penalty += (abs(shoulder_pitch_l) - limit)**2
        shoulder_penalty *= 1.4

        # Ankle Posture Penalty
        ankle_pitch_r = observation[14]
        ankle_pitch_l = observation[15]
        ankle_penalty = 0.0
        limit = 0.2
        if abs(ankle_pitch_r) > limit:
            ankle_penalty += (abs(ankle_pitch_r) - limit)**2
        if abs(ankle_pitch_l) > limit:
            ankle_penalty += (abs(ankle_pitch_l) - limit)**2
        if abs(foot_roll_r) > limit:
            ankle_penalty += (abs(foot_roll_r) - limit)**2
        if abs(foot_roll_l) > limit:
            ankle_penalty += (abs(foot_roll_l) - limit)**2
        ankle_penalty *= 1.4

        # Hip Yaw Penalty
        hip_yaw_r = observation[6]
        hip_yaw_l = observation[7]
        hip_yaw_penalty = 0.0
        limit = 0.1
        if abs(hip_yaw_r) > limit:
            hip_yaw_penalty += (abs(hip_yaw_r) - limit)**2
        if abs(hip_yaw_l) > limit:
            hip_yaw_penalty += (abs(hip_yaw_l) - limit)**2
        hip_yaw_penalty *= 1.4
        
        # Action Smoothness Penalty
        action_smoothness_penalty = 0.1 * np.sum(np.square(action - self.last_action))

        # --- NEW: Knee Bend Penalty ---
        # This penalizes the agent for bending its knees too much, which causes the "sitting" posture.
        # LegLowerR (ID 13) is index 12, LegLowerL (ID 14) is index 13.
        knee_r = observation[12]
        knee_l = observation[13]
        # A straight leg has a knee angle near 0. A bent knee has a large absolute angle.
        # We penalize the sum of the absolute angles to encourage straight legs.
        knee_bend_penalty = 1.0 * (abs(knee_r) + abs(knee_l))


        # Combine all rewards and penalties
        reward = (balance_reward + 
                  foot_symmetry_reward - 
                  movement_penalty - 
                  energy_penalty -
                  stance_width_penalty -
                  shoulder_penalty -
                  action_smoothness_penalty -
                  ankle_penalty -
                  hip_yaw_penalty -
                  knee_bend_penalty) # Subtract the new knee penalty
        
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