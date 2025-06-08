import gymnasium as gym
from stable_baselines3 import TD3
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.logger import configure
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from darwin_env import DarwinOPEnv
import numpy as np
import os
from typing import Optional, Tuple, Dict, Any

# --- Custom Callback for Curriculum Learning and Early Stopping ---
class TrainingCallback(BaseCallback):
    def __init__(self, check_freq: int, goal_distance: float, verbose: int = 1):
        super(TrainingCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.goal_distance = goal_distance
        self.best_distance = 0.0

    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:
            current_x_list = self.training_env.get_attr('current_x')
            if not current_x_list:
                return True
            
            current_x = current_x_list[0]
            
            if self.verbose > 0:
                print(f"Callback check: Current distance = {current_x:.2f} m (Best: {self.best_distance:.2f} m)")

            # Check for new best distance
            if current_x > self.best_distance:
                self.best_distance = current_x
                # Save intermediate model when improving
                if current_x > 0.5: # Only save after some progress
                    self.model.save(f"{MODEL_PATH.replace('.zip', f'_best_{current_x:.1f}m.zip')}")

            # Check for goal achievement
            if self.best_distance >= self.goal_distance:
                if self.verbose > 0:
                    print(f"\n--- Goal Reached! Walked {self.best_distance:.2f} meters. ---")
                self.model.save(MODEL_PATH.replace(".zip", "_goal_reached.zip"))
                return False
              
        return True

# --- Environment Wrapper for Curriculum Learning ---
class CurriculumWrapper(gym.Wrapper):
    def __init__(self, env):
        super(CurriculumWrapper, self).__init__(env)
        self.current_max_distance = 0.5  # Start with small distance target
        self.distance_increment = 0.5    # Increase target by this amount
        self.success_threshold = 0.8     # Must reach this % of target to advance
        
    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        
        # Early termination with bonus if exceeding current allowed distance
        if self.env.current_x > self.current_max_distance:
            terminated = True
            reward += 10.0 # Bonus for reaching the intermediate distance target
        
        return obs, reward, terminated, truncated, info
        
    def reset(self, **kwargs):
        # Increase distance target if previous target was reached
        if self.env.current_x >= self.current_max_distance * self.success_threshold:
            self.current_max_distance += self.distance_increment
            print(f"\n--- Level Up! New distance target: {self.current_max_distance:.1f}m ---")
            
        return self.env.reset(**kwargs)

# --- Environment Creation Function ---
def make_env():
    env = DarwinOPEnv(server_ip='127.0.0.1')
    # env = CurriculumWrapper(env) # <-- You can uncomment this to enable curriculum learning
    env = Monitor(env) 
    return env

# --- Main Execution Block ---
if __name__ == '__main__':
    # --- Configuration ---
    MODEL_PATH = "td3_darwin_stable_v3.zip"
    TENSORBOARD_LOG_DIR = "./td3_darwin_tensorboard_stable_v3/"

    # --- Create Vectorized Environment ---
    # Use DummyVecEnv for stable, single-process training
    env = DummyVecEnv([make_env])

    # --- Agent Hyperparameters ---
    n_actions = env.action_space.shape[-1]
    action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

    # --- Create the Callback ---
    training_callback = TrainingCallback(check_freq=5000, goal_distance=4.0)

    # --- Create or Load Model ---
    if os.path.exists(MODEL_PATH):
        print(f"Found saved model at {MODEL_PATH}. Loading to continue training.")
        model = TD3.load(MODEL_PATH, env=env)
        new_logger = configure(TENSORBOARD_LOG_DIR, ["stdout", "csv", "tensorboard"])
        model.set_logger(new_logger)
    else:
        print("Creating new optimized TD3 agent.")
        model = TD3(
            "MlpPolicy",
            env,
            action_noise=action_noise,
            verbose=1,
            tensorboard_log=TENSORBOARD_LOG_DIR,
            buffer_size=200_000,
            batch_size=128,
            gamma=0.99,
            learning_rate=1e-4,
            tau=0.005,
            learning_starts=10000,
            device='auto'
        )

    # --- Training Loop ---
    try:
        print("\nStarting training...")
        model.learn(
            total_timesteps=2_000_000,
            reset_num_timesteps=False,
            log_interval=10,
            callback=training_callback,
            tb_log_name="TD3_Stable"
        )
    finally:
        print("\nTraining finished or interrupted. Saving final model...")
        model.save(MODEL_PATH)
        print(f"Model saved to {MODEL_PATH}")
        env.close()