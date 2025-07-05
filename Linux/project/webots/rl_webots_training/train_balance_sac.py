import os
import numpy as np
# --- MODIFICATION: Import the SAC algorithm ---
from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import BaseCallback

# Import the specialized environment for balancing
from darwin_env_balance import DarwinOPBalanceEnv

# --- Custom Callback for Balancing ---
# This callback stops training when a high average reward is reached.
class BalanceGoalCallback(BaseCallback):
    def __init__(self, check_freq: int, reward_threshold: float, verbose=1):
        super(BalanceGoalCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.reward_threshold = reward_threshold

    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:
            if len(self.model.ep_info_buffer) > 0:
                mean_reward = np.mean([ep_info['r'] for ep_info in self.model.ep_info_buffer])
                if self.verbose > 0:
                    print(f"Callback check: Mean reward = {mean_reward:.2f} / {self.reward_threshold}")
                
                if mean_reward >= self.reward_threshold:
                    print(f"\n--- BALANCING GOAL REACHED! Mean reward of {mean_reward:.2f} achieved. ---")
                    print("Stopping training and saving balance model.")
                    self.model.save(MODEL_PATH.replace(".zip", "_goal_reached.zip"))
                    return False
        return True

# --- Main Training Execution ---
if __name__ == '__main__':
    NUM_ENVIRONMENTS = 2
    STARTING_PORT = 1234
    
    # Model will be saved with a new name for the SAC agent
    MODEL_PATH = "balance_actor_sac.zip" 
    TENSORBOARD_LOG_DIR = "./td3_darwin_tensorboard_balance_sac/"
    
    print(f"Starting BALANCE training with SAC and {NUM_ENVIRONMENTS} environments.")

    # Create the vectorized environment using the Balance-specific class
    env_fns = [lambda i=i: Monitor(DarwinOPBalanceEnv(port=STARTING_PORT + i)) for i in range(NUM_ENVIRONMENTS)]
    env = SubprocVecEnv(env_fns)

    # Callback will stop when the mean reward is consistently high
    goal_callback = BalanceGoalCallback(check_freq=5000, reward_threshold=600.0)

    if os.path.exists(MODEL_PATH):
        print(f"Found saved SAC balance model at {MODEL_PATH}. Loading to continue training.")
        model = SAC.load(MODEL_PATH, env=env)
    else:
        print("No saved model found. Creating new SAC agent for balancing.")
        # --- MODIFICATION: Instantiate the SAC model ---
        # Note: We do not pass action_noise to SAC.
        model = SAC(
            "MlpPolicy", 
            env, 
            verbose=1, 
            tensorboard_log=TENSORBOARD_LOG_DIR,
            buffer_size=1_000_000,
            batch_size=256,
            gamma=0.99,
            learning_rate=3e-4, # SAC often works well with a slightly higher learning rate
            tau=0.005,
            learning_starts=10000
        )

    try:
        model.learn(total_timesteps=2_000_000, log_interval=10, callback=goal_callback)
    except KeyboardInterrupt:
        print("\nTraining interrupted by user.")
    finally:
        print("\nSaving final SAC balance model...")
        model.save(MODEL_PATH)
        print(f"Model saved to {MODEL_PATH}")
        env.close()