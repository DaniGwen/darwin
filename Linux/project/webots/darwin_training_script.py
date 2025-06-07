from stable_baselines3 import PPO
from darwin_env import DarwinOPEnv # Import your custom environment

# --- 1. Create the Environment ---
# This will connect to your C++ server when created.
# Make sure the C++ server is running on the Raspberry Pi FIRST.
env = DarwinOPEnv(server_ip='192.168.1.100')

# --- 2. Create the RL Agent ---
# We'll use the PPO (Proximal Policy Optimization) algorithm, a popular choice.
# The "MlpPolicy" is a standard multi-layer perceptron (neural network).
model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./ppo_darwin_tensorboard/")

# --- 3. Train the Agent ---
# This command starts the training loop.
# It will run for 1,000,000 steps, but you can change this.
# It will automatically save checkpoints and log training progress.
model.learn(total_timesteps=1_000_000)

# --- 4. Save the Final Model ---
# This saves the trained neural network policy.
model.save("ppo_darwin_model")

# You can later load it with: model = PPO.load("ppo_darwin_model")

env.close()