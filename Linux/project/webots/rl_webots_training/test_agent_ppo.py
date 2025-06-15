import os
import numpy as np
# --- MODIFICATION: Import the PPO algorithm ---
from stable_baselines3 import PPO

# Import the environment the model was trained on
from darwin_env_balance import DarwinOPBalanceEnv

# --- Main Testing Execution ---
if __name__ == '__main__':
    # --- Configuration ---
    # This should match the port your Webots instance is listening on
    PORT = 1234

    # --- Interactive Model Selection ---
    while True:
        model_path = input("Enter the path to the trained PPO .zip model to test: ")
        if os.path.exists(model_path):
            break
        else:
            print(f"Error: File not found at '{model_path}'. Please check the path and try again.")

    # --- Create the Environment ---
    print(f"Connecting to Webots on port {PORT}...")
    env = DarwinOPBalanceEnv(port=PORT)

    # --- Load the Trained Model ---
    print(f"Loading model from {model_path}...")
    try:
        # --- MODIFICATION: Load the model as a PPO agent ---
        model = PPO.load(model_path, env=env)
        print("Model loaded successfully as a PPO agent.")
    except Exception as e:
        print(f"Fatal error loading model: {e}")
        env.close()
        exit()

    # --- Inference Loop ---
    obs, _ = env.reset()
    
    print("\nStarting inference loop. Press Ctrl+C to stop.")
    try:
        while True:
            # Get the best action from the model (deterministic=True is crucial for testing)
            action, _states = model.predict(obs, deterministic=True)
            
            # Interact with the environment
            obs, reward, terminated, truncated, info = env.step(action)
            
            # Reset if the episode ends
            if terminated or truncated:
                print("Episode finished. Resetting environment...")
                obs, _ = env.reset()

    except KeyboardInterrupt:
        print("\nInference stopped by user.")
    finally:
        print("Closing environment.")
        env.close()