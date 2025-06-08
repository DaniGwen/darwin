from stable_baselines3 import TD3
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.logger import configure
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
from stable_baselines3.common.monitor import Monitor
from darwin_env import DarwinOPEnv
import numpy as np
import os
from typing import Optional, Tuple, Dict, Any

# --- Custom Callback for Curriculum Learning and Early Stopping ---
class TrainingCallback(BaseCallback):
    def __init__(self, 
                 check_freq: int, 
                 goal_distance: float,
                 verbose: int = 1):
        super(TrainingCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.goal_distance = goal_distance
        self.best_distance = 0.0
        self.patience = 20  # Number of checks without improvement before early stopping
        self.no_improvement_count = 0

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
                self.no_improvement_count = 0
                
                # Save intermediate model when improving
                if current_x > 0.5:  # Only save after some progress
                    self.model.save(f"{MODEL_PATH.replace('.zip', f'_best_{current_x:.1f}m.zip')")
            else:
                self.no_improvement_count += 1

            # Check for goal achievement
            if current_x >= self.goal_distance:
                if self.verbose > 0:
                    print(f"\n--- Goal Reached! Walked {current_x:.2f} meters. ---")
                self.model.save(MODEL_PATH.replace(".zip", "_goal_reached.zip"))
                return False
                
            # Early stopping if no improvement
            if self.no_improvement_count >= self.patience and self.best_distance > 0.3:
                if self.verbose > 0:
                    print(f"\nEarly stopping - no improvement for {self.patience} checks")
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
        
        # Early termination if exceeding current allowed distance
        if self.env.current_x > self.current_max_distance:
            terminated = True
            reward += 10.0  # Bonus for reaching distance target
            
        return obs, reward, terminated, truncated, info
        
    def reset(self, **kwargs):
        # Increase distance target if previous target was reached
        if self.env.current_x >= self.current_max_distance * self.success_threshold:
            self.current_max_distance += self.distance_increment
            if self.verbose > 0:
                print(f"\nIncreasing distance target to {self.current_max_distance:.1f}m")
                
        return self.env.reset(**kwargs)

# --- Create Parallel Environments ---
def make_env(rank: int = 0) -> gym.Env:
    def _init() -> gym.Env:
        env = DarwinOPEnv(server_ip='127.0.0.1')
        env = CurriculumWrapper(env)
        env = Monitor(env)  # For logging
        return env
    return _init

# --- Configuration ---
MODEL_PATH = "td3_darwin_optimized.zip"
TENSORBOARD_LOG_DIR = "./td3_darwin_tensorboard_optimized/"
NUM_ENVS = 4  # Number of parallel environments

# --- Create Vectorized Environment ---
env = SubprocVecEnv([make_env(i) for i in range(NUM_ENVS)])

# --- Agent Hyperparameters ---
n_actions = env.action_space.shape[-1]
action_noise = NormalActionNoise(
    mean=np.zeros(n_actions), 
    sigma=0.1 * np.ones(n_actions)  # Increased exploration noise
)

# --- Create the Callback ---
training_callback = TrainingCallback(
    check_freq=1000, 
    goal_distance=4.0
)

# --- Check for Existing Model ---
if os.path.exists(MODEL_PATH):
    print(f"Found saved model at {MODEL_PATH}. Loading to continue training.")
    model = TD3.load(MODEL_PATH, env=env)
    
    new_logger = configure(TENSORBOARD_LOG_DIR, ["stdout", "csv", "tensorboard"])
    model.set_logger(new_logger)
else:
    # --- Create Optimized TD3 Model ---
    print("Creating new optimized TD3 agent.")
    model = TD3(
        "MlpPolicy",
        env,
        action_noise=action_noise,
        verbose=1,
        tensorboard_log=TENSORBOARD_LOG_DIR,
        buffer_size=500_000,      # Smaller buffer for faster learning
        batch_size=128,           # Smaller batches
        gamma=0.98,               # Slightly shorter horizon
        learning_rate=3e-4,       # Increased learning rate
        tau=0.01,                 # Faster target network updates
        learning_starts=5000,      # Start learning earlier
        policy_delay=4,           # More frequent policy updates
        target_policy_noise=0.2,  # More target exploration
        target_noise_clip=0.5,
        device='auto'             # Use GPU if available
    )

# --- Training Loop ---
try:
    print("\nStarting training...")
    model.learn(
        total_timesteps=2_000_000,
        reset_num_timesteps=False,
        log_interval=10,
        callback=training_callback,
        tb_log_name="TD3_optimized"
    )

finally:
    print("\nTraining finished or interrupted. Saving final model...")
    model.save(MODEL_PATH)
    print(f"Model saved to {MODEL_PATH}")
    env.close()