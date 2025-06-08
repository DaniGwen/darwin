from stable_baselines3 import PPO
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
    # For resuming, we load the model and ensure the environment is the same
    model = PPO.load(MODEL_PATH, env=env)
    # It's good practice to reset the logger to start a new logging session in TensorBoard
    model.set_logger(None) 
    model.set_logger(type(model.logger)(folder=TENSORBOARD_LOG_DIR, log_suffix=""))
else:
    # --- Create a new PPO model from scratch ---
    # We are switching back to PPO for its stability
    print("No saved model found. Creating a new PPO agent.")
    model = PPO(
        "MlpPolicy", 
        env, 
        verbose=1, 
        tensorboard_log=TENSORBOARD_LOG_DIR,
        n_steps=2048, # The number of steps to run for each environment per update
        batch_size=64, # The number of samples per gradient update
        n_epochs=10, # The number of epochs when optimizing the surrogate loss
        gamma=0.99, # Discount factor
        gae_lambda=0.95, # Factor for trade-off of bias vs variance for GAE
        clip_range=0.2, # Clipping parameter for PPO
        ent_coef=0.0, # Entropy coefficient for exploration. PPO handles exploration well.
        learning_rate=3e-4 # Learning rate for the optimizer
    )


try:
    # --- Train the Agent ---
    # We set reset_num_timesteps=False to ensure that if we are resuming training,
    # the total timesteps counter doesn't reset to zero.
    model.learn(total_timesteps=1_000_000, reset_num_timesteps=False)

finally:
    # --- Save the Final Model ---
    # This block will run even if you interrupt the script with CTRL+C
    print("\nSaving final model...")
    model.save(MODEL_PATH)
    print(f"Model saved to {MODEL_PATH}")
    env.close()