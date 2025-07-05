import os
import numpy as np
from stable_baselines3 import TD3
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import BaseCallback

# Import the specialized environment for walking
from darwin_env_walk_forward import DarwinOPWalkEnv

# --- Custom Callback for Walking ---
# This callback stops training when the robot walks a certain distance.
class WalkGoalCallback(BaseCallback):
    def __init__(self, check_freq: int, goal_distance: float, verbose=1):
        super(WalkGoalCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.goal_distance = goal_distance

    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:
            # Get current_x from the first environment for logging
            current_x_list = self.training_env.get_attr('current_x')
            if not current_x_list: return True
            current_x = current_x_list[0]
            
            if self.verbose > 0:
                print(f"Callback check: Walk distance = {current_x:.2f}m / {self.goal_distance}m")

            if current_x >= self.goal_distance:
                print(f"\n--- WALKING GOAL REACHED! Walked {current_x:.2f} meters. ---")
                self.model.save(MODEL_PATH.replace(".zip", "_goal_reached.zip"))
                return False
        return True

# --- Main Training Execution ---
if __name__ == '__main__':
    NUM_ENVIRONMENTS = 2
    STARTING_PORT = 1234
    
    MODEL_PATH = "walk_actor.zip"
    TENSORBOARD_LOG_DIR = "./td3_darwin_tensorboard_walk/"
    
    print(f"Starting WALK training with {NUM_ENVIRONMENTS} environments.")

    env_fns = [lambda i=i: Monitor(DarwinOPWalkEnv(port=STARTING_PORT + i)) for i in range(NUM_ENVIRONMENTS)]
    env = SubprocVecEnv(env_fns)

    action_noise = NormalActionNoise(mean=np.zeros(18), sigma=0.05 * np.ones(18))

    goal_callback = WalkGoalCallback(check_freq=1000, goal_distance=5.0)

    if os.path.exists(MODEL_PATH):
        print(f"Found saved walk model at {MODEL_PATH}. Loading to continue training.")
        model = TD3.load(MODEL_PATH, env=env)
    else:
        print("No saved model found. Creating new TD3 agent for walking.")
        model = TD3("MlpPolicy", env, action_noise=action_noise, verbose=1, tensorboard_log=TENSORBOARD_LOG_DIR, learning_starts=10000)

    try:
        model.learn(total_timesteps=3_000_000, log_interval=10, callback=goal_callback)
    except KeyboardInterrupt:
        print("\nTraining interrupted by user.")
    finally:
        print("\nSaving final walk model...")
        model.save(MODEL_PATH)
        print(f"Model saved to {MODEL_PATH}")
        env.close()