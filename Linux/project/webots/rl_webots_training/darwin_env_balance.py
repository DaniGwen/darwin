from darwin_env_base import DarwinOPEnvBase
import numpy as np

class OptimizedDarwinOPBalanceEnv(DarwinOPEnvBase):
    """
    An optimized environment for training the Darwin-OP robot to balance.
    
    Key optimizations include:
    1.  Smoother, exponential reward functions instead of linear penalties with cutoffs.
    2.  Consolidated and simplified reward terms to avoid conflicting goals.
    3.  Reduced action scaling for finer motor control and stability.
    4.  Re-tuned weights to prioritize core balancing over secondary posture goals.
    """
    def __init__(self, **kwargs):
        super(OptimizedDarwinOPBalanceEnv, self).__init__(**kwargs)
        self.target_height = 0.33
        self.last_action = np.zeros(self.action_space.shape, dtype=np.float32)

    # --- OPTIMIZATION 1: Finer Action Control ---
    # The action space is scaled down. Instead of mapping [-1, 1] to [-1, 1] radians,
    # we map to a smaller range, e.g., [-0.4, 0.4] radians. This prevents
    # jerky movements and encourages finer control.
    def send_action(self, action):
        """Sends the scaled action to the simulator."""
        try:
            # Scale factor reduced from 1.0 to 0.4 for more stable actions
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
            return np.zeros(self.observation_space.shape), -10.0, True, False, {}

        roll, pitch = observation[18], observation[19]
        self.current_x, height = observation[21], observation[23]

        # --- OPTIMIZATION 2: Smoother, Exponential Rewards ---
        # Using exponential functions provides a smooth gradient for the agent.
        # The reward is highest at the target and gracefully decreases.
        
        # Core balance rewards
        height_reward = np.exp(-25.0 * abs(height - self.target_height))
        pitch_reward = np.exp(-5.0 * abs(pitch))
        roll_reward = np.exp(-5.0 * abs(roll))
        
        # The primary reward is for being upright and at the correct height.
        balance_reward = 3.0 * (height_reward + pitch_reward + roll_reward)

        # --- OPTIMIZATION 3: Simplified and Consolidated Posture Control ---
        
        # Reward for a stable, slightly athletic knee bend.
        # This replaces the previous "stand_tall_reward" and "knee_bend_penalty".
        # It encourages a slight bend (around 0.1 rad) which is more stable than locked knees.
        knee_r, knee_l = observation[12], observation[13]
        target_knee_bend = 0.1
        knee_posture_reward = 1.5 * np.exp(-15.0 * (abs(knee_r - target_knee_bend) + abs(knee_l - target_knee_bend)))

        # Reward for symmetrical foot placement
        foot_roll_r, foot_roll_l = observation[16], observation[17]
        foot_symmetry_reward = 0.5 * np.exp(-5.0 * abs(foot_roll_r - foot_roll_l))

        # --- OPTIMIZATION 4: Re-tuned Penalties ---

        # Penalty for excessive forward/backward movement (reduced to allow small adjustments)
        movement_penalty = 5.0 * abs(self.current_x - self.last_x_position)
        
        # Penalty for energy usage and jerky movements
        energy_penalty = 0.001 * np.sum(np.square(action))
        action_smoothness_penalty = 0.1 * np.sum(np.square(action - self.last_action))

        # Combine all reward and penalty terms
        reward = (balance_reward + 
                  knee_posture_reward + 
                  foot_symmetry_reward -
                  movement_penalty - 
                  energy_penalty - 
                  action_smoothness_penalty)
        
        self.last_action = action
        self.last_x_position = self.current_x

        # --- OPTIMIZATION 5: Clearer Termination Conditions ---
        # Increased penalty for falling to make it a more significant event.
        terminated = (height < 0.22 or abs(pitch) > 1.0 or abs(roll) > 1.0 or self.episode_steps > 1000)
        if terminated:
            reward = -10.0 # A large, fixed penalty for failure

        return observation, reward, terminated, False, {}

    def reset(self, seed=None, options=None):
        self.last_action = np.zeros(self.action_space.shape, dtype=np.float32)
        return super().reset(seed=seed, options=options)