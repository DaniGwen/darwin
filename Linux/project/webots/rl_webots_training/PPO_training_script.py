from stable_baselines3 import PPO
from stable_baselines3.common.logger import configure
from darwin_env import DarwinOPEnv 
import os

# --- Configuration ---
MODEL_PATH = "ppo_darwin_model.zip"
TENSORBOARD_LOG_DIR = "./ppo_darwin_tensorboard/"

# --- Create the Environment ---
env = DarwinOPEnv(server_ip='127.0.0.1')

# --- Check if a saved model exists ---
if os.path.exists(MODEL_PATH):
    print(f"Found saved model at {MODEL_PATH}. Loading to continue training.")
    model = PPO.load(MODEL_PATH, env=env)
    
    # Correctly configure a new logger for the new training session
    new_logger = configure(TENSORBOARD_LOG_DIR, ["stdout", "csv", "tensorboard"])
    model.set_logger(new_logger)
else:
    # --- Create a new PPO model with more stable hyperparameters ---
    print("No saved model found. Creating a new PPO agent with stable settings.")
    model = PPO(
        "MlpPolicy", 
        env, 
        verbose=1, 
        tensorboard_log=TENSORBOARD_LOG_DIR,
        n_steps=4096,          # Collect more data before each update
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.001,        # Add a small amount of exploration
        learning_rate=1e-4     # Use a smaller, more stable learning rate
    )

try:
    # --- Train the Agent ---
    # Set total_timesteps to a large number for extended training
    model.learn(total_timesteps=2_000_000, reset_num_timesteps=False)

finally:
    # --- Save the Final Model ---
    print("\nSaving final model...")
    model.save(MODEL_PATH)
    print(f"Model saved to {MODEL_PATH}")
    env.close()