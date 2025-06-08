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
    # Load the model
    model = PPO.load(MODEL_PATH, env=env)
    
    # --- CORRECTED LOGGER SETUP ---
    # Create and set a new logger for the new training session
    new_logger = configure(TENSORBOARD_LOG_DIR, ["stdout", "csv", "tensorboard"])
    model.set_logger(new_logger)
else:
    # --- Create a new PPO model from scratch ---
    print("No saved model found. Creating a new PPO agent.")
    model = PPO(
        "MlpPolicy", 
        env, 
        verbose=1, 
        tensorboard_log=TENSORBOARD_LOG_DIR,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.0,
        learning_rate=3e-4
    )

try:
    # --- Train the Agent ---
    # reset_num_timesteps=False ensures that if we are resuming training,
    # the total timesteps counter doesn't reset to zero.
    model.learn(total_timesteps=1_000_000, reset_num_timesteps=False)

finally:
    # --- Save the Final Model ---
    # This block will run even if you interrupt the script with CTRL+C
    print("\nSaving final model...")
    model.save(MODEL_PATH)
    print(f"Model saved to {MODEL_PATH}")
    env.close()