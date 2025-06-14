import os
import numpy as np
from stable_baselines3 import TD3

# We will use the balance environment as that's what the model was trained on
from darwin_env_balance import DarwinOPBalanceEnv

# --- Main Testing Execution ---
if __name__ == '__main__':
    # --- Configuration ---
    # This should match the port your Webots instance is listening on
    PORT = 1234

    # --- Interactive Model Selection ---
    while True:
        model_path = input("Enter the path to the trained .zip model to test (e.g., balance_actor_goal_reached.zip): ")
        if os.path.exists(model_path):
            break
        else:
            print(f"Error: File not found at '{model_path}'. Please check the path and try again.")

    print(f"Connecting to Webots on port {PORT}...")
    env = DarwinOPBalanceEnv(port=PORT)

    # --- Load the Trained Model ---
    print(f"Loading model from {model_path}...")
    try:
        model = TD3.load(model_path, env=env)
        print("Model loaded successfully.")
    except Exception as e:
        print(f"Fatal error loading model: {e}")
        env.close()
        exit()

    # --- Inference Loop ---
    # Get the initial observation from the environment
    obs, _ = env.reset()
    
    print("\nStarting inference loop. Press Ctrl+C to stop.")
    try:
        while True:
            # Get the best action from the model
            # deterministic=True ensures we don't use random exploration noise
            action, _states = model.predict(obs, deterministic=True)
            
            # Send the action to the environment and get the new state
            obs, reward, terminated, truncated, info = env.step(action)
            
            # If the episode ends (robot falls), reset the environment
            if terminated or truncated:
                print("Episode finished. Resetting environment...")
                obs, _ = env.reset()

    except KeyboardInterrupt:
        print("\nInference stopped by user.")
    finally:
        print("Closing environment.")
        env.close()