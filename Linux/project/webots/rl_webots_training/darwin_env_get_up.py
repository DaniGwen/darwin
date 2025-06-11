from darwin_env_base import DarwinOPEnvBase
import numpy as np

class DarwinOPGetUpEnv(DarwinOPEnvBase):
    def __init__(self, **kwargs):
        super(DarwinOPGetUpEnv, self).__init__(**kwargs)
        # The goal is to reach this height from a fallen state
        self.stand_up_height_target = 0.30 

    def step(self, action):
        self.episode_steps += 1

        observation = self.send_action(action)
        if observation is None:
            return np.zeros(self.observation_space.shape), -10.0, True, False, {}

        height = observation[23]
        
        # --- Get-Up Specific Reward ---
        # The primary reward is for increasing the torso height.
        # This encourages any action that pushes the robot off the ground.
        height_reward = height * 10.0

        # Small penalty for energy to encourage efficiency
        energy_penalty = 0.001 * np.sum(np.square(action))
        
        reward = height_reward - energy_penalty

        # Termination conditions
        terminated = self.episode_steps > 300 # Give it a fixed time to try and get up
        if height >= self.stand_up_height_target:
            print(f"Stand-up successful on port {self.port}!")
            reward += 100.0 # Huge bonus for success
            terminated = True
            
        return observation, reward, terminated, False, {}
    
    def reset(self, seed=None, options=None):
        """
        Special reset for the get-up task. We need to make the robot fall over first.
        This is a simple way; a more robust method might apply a force in simulation.
        """
        # First, do a standard reset to get the robot standing
        obs, info = super().reset(seed=seed)
        
        # Now, command the robot to a fallen pose (e.g., squatting very low)
        fall_action = np.zeros(18)
        # Bend knees and hips
        fall_action[12] = 2.0  # R LegLower
        fall_action[13] = -2.0 # L LegLower
        fall_action[10] = -1.0 # R LegUpper
        fall_action[11] = 1.0  # L LegUpper

        # Hold the fall pose for a few steps to ensure it's on the ground
        for _ in range(20):
            obs, _, _, _, _ = self.step(fall_action)

        print(f"Robot is now in a fallen state on port {self.port}. Beginning get-up attempt.")
        self.episode_steps = 0 # Reset episode steps after falling
        return obs, info