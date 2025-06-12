import os
import numpy as np
from stable_baselines3 import PPO  # PPO is often a good choice for vision-based tasks
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import BaseCallback

# Import the specialized environment for grasping
from darwin_env_grasp import DarwinOPGraspEnv

# --- Custom Callback for Grasping ---
# This callback stops training when a high reward is achieved,
# indicating a successful grasp.
class GraspGoalCallback(BaseCallback):
    def __init__(self, check_freq: int, reward_threshold: float, verbose=1):
        super(GraspGoalCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.reward_threshold = reward_threshold

    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:
            if len(self.model.ep_info_buffer) > 0:
                # A successful grasp has a huge reward bonus (>90)
                recent_rewards = [ep_info['r'] for ep_info in self.model.ep_info_buffer]
                if self.verbose > 0:
                    print(f"Callback check: Max recent reward = {np.max(recent_rewards):.2f} / {self.reward_threshold}")
                
                if np.max(recent_rewards) >= self.reward_threshold:
                    print(f"\n--- GRASP GOAL REACHED! Successfully reached for an object. ---")
                    self.model.save(MODEL_PATH.replace(".zip", "_goal_reached.zip"))
                    return False
        return True

# --- Main Training Execution ---
if __name__ == '__main__':
    # Vision-based training is slow, so using multiple environments is highly recommended.
    NUM_ENVIRONMENTS = 2  # Adjust based on your computer's capability
    STARTING_PORT = 1234
    
    MODEL_PATH = "grasp_actor.zip"
    TENSORBOARD_LOG_DIR = "./td3_darwin_tensorboard_grasp/"
    
    print(f"Starting GRASP training with {NUM_ENVIRONMENTS} environments.")

    # Create the vectorized environment using the Grasp-specific class
    env_fns = [lambda i=i: Monitor(DarwinOPGraspEnv(port=STARTING_PORT + i)) for i in range(NUM_ENVIRONMENTS)]
    env = SubprocVecEnv(env_fns)

    # The reward threshold should be high to catch the +100 bonus for success
    goal_callback = GraspGoalCallback(check_freq=500, reward_threshold=90.0)

    if os.path.exists(MODEL_PATH):
        print(f"Found saved grasp model at {MODEL_PATH}. Loading to continue training.")
        # Use the CnnPolicy because we are learning from images
        model = PPO.load(MODEL_PATH, env=env, policy="CnnPolicy")
    else:
        print("No saved model found. Creating new PPO agent for grasping.")
        # We must use the "CnnPolicy" for vision-based learning.
        model = PPO(
            "CnnPolicy",
            env,
            verbose=1,
            tensorboard_log=TENSORBOARD_LOG_DIR,
            n_steps=1024,
            batch_size=64,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.0,
            vf_coef=0.5,
            max_grad_norm=0.5,
        )

    try:
        # Vision-based learning needs many more timesteps
        model.learn(total_timesteps=5_000_000, log_interval=1, callback=goal_callback)
    except KeyboardInterrupt:
        print("\nTraining interrupted by user.")
    finally:
        print("\nSaving final grasp model...")
        model.save(MODEL_PATH)
        print(f"Model saved to {MODEL_PATH}")
        env.close()

