from stable_baselines3 import TD3
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.logger import configure
from stable_baselines3.common.callbacks import BaseCallback
from darwin_env import DarwinOPEnv 
import numpy as np
import os

# --- Custom Callback to Stop Training on Goal Achievement ---
class GoalCallback(BaseCallback):
    def __init__(self, check_freq: int, goal_distance: float, verbose=1):
        super(GoalCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.goal_distance = goal_distance

    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:
          current_x_list = self.training_env.get_attr('current_x')
          if not current_x_list:
              return True
          
          current_x = current_x_list[0]
          
          if self.verbose > 0:
              print(f"Callback check: Current distance = {current_x:.2f} m / {self.goal_distance} m")

          if current_x >= self.goal_distance:
              if self.verbose > 0:
                  print(f"\n--- Goal Reached! Walked {current_x:.2f} meters. ---")
                  print("Stopping training and saving model.")
              self.model.save(MODEL_PATH.replace(".zip", "_goal_reached.zip"))
              return False
              
        return True

# --- Configuration ---
MODEL_PATH = "td3_darwin_no_head.zip" # New model name for this experiment
TENSORBOARD_LOG_DIR = "./td3_darwin_tensorboard_no_head/"

# --- Create the Environment ---
env = DarwinOPEnv(server_ip='127.0.0.1')

# --- Agent Hyperparameters ---
# --- MODIFIED: Action noise dimension is now 18 ---
n_actions = env.action_space.shape[-1]
action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.05 * np.ones(n_actions))

# --- Create the Callback ---
goal_callback = GoalCallback(check_freq=1000, goal_distance=4.0)

# --- Check if a saved model exists ---
if os.path.exists(MODEL_PATH):
    print(f"Found saved model at {MODEL_PATH}. Loading to continue training.")
    model = TD3.load(MODEL_PATH, env=env)
    
    new_logger = configure(TENSORBOARD_LOG_DIR, ["stdout", "csv", "tensorboard"])
    model.set_logger(new_logger)
else:
    # --- Create a new TD3 model ---
    print("No saved model found. Creating new TD3 agent (no head).")
    model = TD3(
        "MlpPolicy",
        env,
        action_noise=action_noise,
        verbose=1,
        tensorboard_log=TENSORBOARD_LOG_DIR,
        buffer_size=1_000_000,
        batch_size=256,
        gamma=0.99,
        learning_rate=1e-4,
        tau=0.005,
        learning_starts=10000
    )

try:
    model.learn(
        total_timesteps=2_000_000, 
        reset_num_timesteps=False, 
        log_interval=10, 
        callback=goal_callback
    )

finally:
    print("\nTraining finished or interrupted. Saving final model...")
    model.save(MODEL_PATH)
    print(f"Model saved to {MODEL_PATH}")
    env.close()