import torch
from stable_baselines3 import TD3

# --- Configuration based on your previous files ---
MODEL_PATH = "td3_darwin_no_head.zip"  # The file saved by your training script
ONNX_OUTPUT_PATH = "darwin_actor.onnx"

# The dimensions must match your C++ supervisor file
OBSERVATION_DIM = 24  # 18 motors + 3 IMU + 3 position
ACTION_DIM = 18       # 18 motors

# 1. Load the trained Stable-Baselines3 model
print(f"Loading model from {MODEL_PATH}...")
model = TD3.load(MODEL_PATH, device='cpu')

# 2. Extract the underlying PyTorch actor network.
# For TD3, the deterministic action comes from the 'mu' network.
actor_net = model.actor.mu
actor_net.eval() # Set the network to evaluation mode

# 3. Create a dummy input tensor with the correct shape (1, 24)
dummy_input = torch.randn(1, OBSERVATION_DIM)

# 4. Export the actor network to the ONNX format
print(f"Exporting actor network to {ONNX_OUTPUT_PATH}...")
torch.onnx.export(
    actor_net,
    dummy_input,
    ONNX_OUTPUT_PATH,
    input_names=['observation'],  # Name for the input node
    output_names=['action'],     # Name for the output node
    opset_version=11,
    dynamic_axes={'observation': {0: 'batch_size'}, # Optional: allows for variable batch size
                  'action': {0: 'batch_size'}}
)

print("\nExport complete.")
print(f"The model has been correctly exported to '{ONNX_OUTPUT_PATH}' with:")
print(f"- Input dimension: {OBSERVATION_DIM}")
print(f"- Output dimension: {ACTION_DIM}")