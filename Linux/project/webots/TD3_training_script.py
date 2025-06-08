from stable_baselines3 import TD3
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.logger import configure
from darwin_env import DarwinOPEnv 
import numpy as np
import os

# --- Configuration ---
MODEL_PATH = "td3_darwin_model.zip"
TENSORBOARD_LOG_DIR = "./td3_darwin_tensorboard/"

# --- Create the Environment ---
env = DarwinOPEnv(server_ip='127.0.0.1')

# --- Agent Hyperparameters ---
# Action noise is critical for exploration in off-policy algorithms like TD3
n_actions = env.action_space.shape[-1]
action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

# --- Check if a saved model exists ---
if os.path.exists(MODEL_PATH):
    print(f"Found saved model at {MODEL_PATH}. Loading to continue training.")
    model = TD3.load(MODEL_PATH, env=env)
    
    # Correctly configure a new logger for the new training session
    new_logger = configure(TENSORBOARD_LOG_DIR, ["stdout", "csv", "tensorboard"])
    model.set_logger(new_logger)
else:
    # --- Create a new TD3 model from scratch ---
    print("No saved model found. Creating a new TD3 agent.")
    model = TD3(
        "MlpPolicy",
        env,
        action_noise=action_noise,
        verbose=1,
        tensorboard_log=TENSORBOARD_LOG_DIR,
        buffer_size=1_000_000,   # Size of the replay buffer
        batch_size=256,         # How many samples to use for each update
        gamma=0.99,             # Discount factor
        learning_rate=3e-4,     # Learning rate for actor and critic
        tau=0.005,              # Soft update coefficient for target networks
        learning_starts=10000   # Number of steps to collect before starting training
    )

try:
    # --- Train the Agent ---
    # The total_timesteps is set high for long-term training.
    # reset_num_timesteps=False is important for resuming.
    model.learn(total_timesteps=1_000_000, reset_num_timesteps=False, log_interval=10)

finally:
    # --- Save the Final Model ---
    # This block will run even if you interrupt the script with CTRL+C
    print("\nSaving final model...")
    model.save(MODEL_PATH)
    print(f"Model saved to {MODEL_PATH}")
    env.close()