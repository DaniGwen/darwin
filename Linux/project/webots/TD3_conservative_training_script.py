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
          # Check the learning phase
          current_phase_list = self.training_env.get_attr('learning_phase')
          if not current_phase_list:
              return True
          current_phase = current_phase_list[0]

          # Only check for distance goal if in the 'walk' phase
          if current_phase == 'walk':
              current_x_list = self.training_env.get_attr('current_x')
              if not current_x_list:
                  return True
              
              current_x = current_x_list[0]
              
              if self.verbose > 0:
                  print(f"Callback check (Phase: {current_phase}): Current distance = {current_x:.2f} m / {self.goal_distance} m")

              if current_x >= self.goal_distance:
                  if self.verbose > 0:
                      print(f"\n--- FINAL GOAL REACHED! Walked {current_x:.2f} meters. ---")
                      print("Stopping training and saving model.")
                  self.model.save(MODEL_PATH.replace(".zip", "_goal_reached.zip"))
                  return False
          else:
              if self.verbose > 0:
                  # Provide feedback on the balance phase
                  steps_balanced_list = self.training_env.get_attr('steps_balanced_continuously')
                  balance_duration_list = self.training_env.get_attr('balance_success_duration')
                  if steps_balanced_list and balance_duration_list:
                       print(f"Callback check (Phase: {current_phase}): Balance progress = {steps_balanced_list[0]} / {balance_duration_list[0]} steps")

        return True

# --- Configuration ---
MODEL_PATH = "td3_darwin_no_head.zip" 
TENSORBOARD_LOG_DIR = "./td3_darwin_tensorboard_no_head/"

# --- Create the Environment ---
env = DarwinOPEnv(server_ip='127.0.0.1')

# --- Agent Hyperparameters ---
n_actions = env.action_space.shape[-1]
action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.05 * np.ones(n_actions))

# --- Create the Callback ---
# MODIFIED: goal_distance is now 5.0 meters
goal_callback = GoalCallback(check_freq=1000, goal_distance=5.0)

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