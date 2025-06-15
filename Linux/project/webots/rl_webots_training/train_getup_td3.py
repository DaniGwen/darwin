import os
import numpy as np
from stable_baselines3 import TD3
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import BaseCallback

# Import the specialized environment for getting up
from darwin_env_get_up import DarwinOPGetUpEnv

# --- Custom Callback for Get Up ---
# This callback stops training when a high reward is achieved, indicating a successful stand-up.
class GetUpGoalCallback(BaseCallback):
    def __init__(self, check_freq: int, reward_threshold: float, verbose=1):
        super(GetUpGoalCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.reward_threshold = reward_threshold

    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:
            if len(self.model.ep_info_buffer) > 0:
                # A successful get-up has a huge reward bonus (>100)
                # We check if any recent episode had this high reward.
                recent_rewards = [ep_info['r'] for ep_info in self.model.ep_info_buffer]
                if self.verbose > 0:
                    print(f"Callback check: Max recent reward = {np.max(recent_rewards):.2f} / {self.reward_threshold}")
                
                if np.max(recent_rewards) >= self.reward_threshold:
                    print(f"\n--- GET UP GOAL REACHED! Achieved a stand-up. ---")
                    self.model.save(MODEL_PATH.replace(".zip", "_goal_reached.zip"))
                    return False
        return True

# --- Main Training Execution ---
if __name__ == '__main__':
    NUM_ENVIRONMENTS = 2
    STARTING_PORT = 1234
    
    MODEL_PATH = "standup_actor.zip"
    TENSORBOARD_LOG_DIR = "./td3_darwin_tensorboard_getup/"
    
    print(f"Starting GET UP training with {NUM_ENVIRONMENTS} environments.")

    env_fns = [lambda i=i: Monitor(DarwinOPGetUpEnv(port=STARTING_PORT + i)) for i in range(NUM_ENVIRONMENTS)]
    env = SubprocVecEnv(env_fns)

    action_noise = NormalActionNoise(mean=np.zeros(18), sigma=0.1 * np.ones(18)) # More noise can help exploration

    # The reward threshold should be high to catch the +100 bonus for success
    goal_callback = GetUpGoalCallback(check_freq=1000, reward_threshold=90.0)

    if os.path.exists(MODEL_PATH):
        print(f"Found saved get-up model at {MODEL_PATH}. Loading to continue training.")
        model = TD3.load(MODEL_PATH, env=env)
    else:
        print("No saved model found. Creating new TD3 agent for getting up.")
        model = TD3("MlpPolicy", env, action_noise=action_noise, verbose=1, tensorboard_log=TENSORBOARD_LOG_DIR, learning_starts=10000)

    try:
        model.learn(total_timesteps=1_000_000, log_interval=10, callback=goal_callback)
    except KeyboardInterrupt:
        print("\nTraining interrupted by user.")
    finally:
        print("\nSaving final get-up model...")
        model.save(MODEL_PATH)
        print(f"Model saved to {MODEL_PATH}")
        env.close()