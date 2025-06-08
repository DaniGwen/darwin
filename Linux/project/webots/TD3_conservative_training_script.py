from stable_baselines3 import TD3
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.logger import configure
from darwin_env import DarwinOPEnv 
import numpy as np
import os

# --- Configuration ---
MODEL_PATH = "td3_darwin_model_stable.zip"  # New model name for this experiment
TENSORBOARD_LOG_DIR = "./td3_darwin_tensorboard_stable/"

# --- Create the Environment ---
env = DarwinOPEnv(server_ip='127.0.0.1')

# --- Agent Hyperparameters ---
# We reduce the noise sigma to make exploration less aggressive and more stable.
n_actions = env.action_space.shape[-1]
action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.05 * np.ones(n_actions))

# --- Check if a saved model exists ---
if os.path.exists(MODEL_PATH):
    print(f"Found saved model at {MODEL_PATH}. Loading to continue training.")
    model = TD3.load(MODEL_PATH, env=env)
    
    # Correctly configure a new logger for the new training session
    new_logger = configure(TENSORBOARD_LOG_DIR, ["stdout", "csv", "tensorboard"])
    model.set_logger(new_logger)
else:
    # --- Create a new TD3 model with more stable hyperparameters ---
    print("No saved model found. Creating a new TD3 agent with stable settings.")
    model = TD3(
        "MlpPolicy",
        env,
        action_noise=action_noise,
        verbose=1,
        tensorboard_log=TENSORBOARD_LOG_DIR,
        buffer_size=1_000_000,
        batch_size=256,
        gamma=0.99,
        learning_rate=1e-4,     # Reduced learning rate for more stable updates
        tau=0.005,
        learning_starts=10000
    )

try:
    # --- Train the Agent ---
    model.learn(total_timesteps=1_000_000, reset_num_timesteps=False, log_interval=10)

finally:
    # --- Save the Final Model ---
    print("\nSaving final model...")
    model.save(MODEL_PATH)
    print(f"Model saved to {MODEL_PATH}")
    env.close()