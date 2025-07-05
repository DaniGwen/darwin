import torch
from stable_baselines3 import TD3, PPO, SAC # Import other algorithms you might use
import os

# --- Configuration ---
# The dimensions must match your C++ supervisor file
OBSERVATION_DIM = 24  # 18 motors + 3 IMU + 3 position
ACTION_DIM = 18       # 18 motors

# --- Interactive Model Selection ---
while True:
    model_path = input("Enter the path to the trained model .zip file (e.g., balance_actor.zip): ")
    if os.path.exists(model_path):
        break
    else:
        print(f"Error: File not found at '{model_path}'. Please check the path and try again.")

# Automatically determine the output ONNX file name
base_name = os.path.splitext(model_path)[0]
onnx_output_path = f"{base_name}.onnx"

# --- Model Loading ---
try:
    print(f"Loading model from {model_path}...")
    # We try to load as TD3 first, as it's been the primary algorithm.
    # A more robust script could try to determine the type or ask the user.
    model = TD3.load(model_path, device='cpu')
    # Extract the actor network for TD3
    actor_net = model.actor.mu
    print("Model loaded as TD3.")

except Exception as e:
    print(f"Could not load as TD3 ({e}). Trying to load as PPO...")
    try:
        model = PPO.load(model_path, device='cpu')
        # For PPO, the actor network is part of the policy object
        actor_net = model.policy.actor
        print("Model loaded as PPO.")
    except Exception as e2:
        print(f"Fatal Error: Could not load model as TD3 or PPO: {e2}")
        exit()


# --- ONNX Export ---
actor_net.eval() # Set the network to evaluation mode

# Create a dummy input tensor with the correct shape (1, 24)
dummy_input = torch.randn(1, OBSERVATION_DIM)

print(f"Exporting actor network to {onnx_output_path}...")
try:
    torch.onnx.export(
        actor_net,
        dummy_input,
        onnx_output_path,
        input_names=['observation'],  # Name for the input node
        output_names=['action'],     # Name for the output node
        opset_version=11,
        dynamic_axes={'observation': {0: 'batch_size'}, # Optional
                      'action': {0: 'batch_size'}}
    )

    print("\nExport complete.")
    print(f"Model '{model_path}' correctly exported to '{onnx_output_path}'")
    print(f"- Input dimension: {OBSERVATION_DIM}")
    print(f"- Output dimension: {ACTION_DIM}")

except Exception as e:
    print(f"\nAn error occurred during ONNX export: {e}")

