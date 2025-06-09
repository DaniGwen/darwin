import os
import numpy as np
from stable_baselines3 import TD3
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.logger import configure
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import SubprocVecEnv

from darwin_env import DarwinOPEnv

# --- Custom Callback (Unchanged, but works with VecEnv) ---
class GoalCallback(BaseCallback):
    def __init__(self, check_freq: int, goal_distance: float, verbose=1):
        super(GoalCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.goal_distance = goal_distance

    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:
            current_phase_list = self.training_env.get_attr('learning_phase')
            if not current_phase_list: return True
            
            # Log progress from the first environment
            current_phase = current_phase_list[0]

            if current_phase == 'walk':
                current_x_list = self.training_env.get_attr('current_x')
                if not current_x_list: return True
                current_x = current_x_list[0]
                
                if self.verbose > 0:
                    print(f"Callback check (Env 0, Phase: {current_phase}): Distance = {current_x:.2f}m / {self.goal_distance}m")

                if current_x >= self.goal_distance:
                    print(f"\n--- FINAL GOAL REACHED! Walked {current_x:.2f} meters. ---")
                    print("Stopping training and saving final model.")
                    self.model.save(MODEL_PATH.replace(".zip", "_goal_reached.zip"))
                    return False
            else:
                steps_balanced_list = self.training_env.get_attr('steps_balanced_continuously')
                balance_duration_list = self.training_env.get_attr('balance_success_duration')
                if steps_balanced_list and balance_duration_list:
                    print(f"Callback check (Env 0, Phase: {current_phase}): Balance progress = {steps_balanced_list[0]}/{balance_duration_list[0]} steps")
        return True

# --- Main Training Execution ---
if __name__ == '__main__':
    # --- Parallel Training Configuration ---
    NUM_ENVIRONMENTS = 2 # <<-- SET THE NUMBER OF WEBOTS INSTANCES TO RUN
    STARTING_PORT = 1234
    
    MODEL_PATH = "td3_darwin_no_head.zip"
    TENSORBOARD_LOG_DIR = "./td3_darwin_tensorboard_parallel/"
    
    print(f"Starting parallel training with {NUM_ENVIRONMENTS} environments.")

    # --- Create the Vectorized Environment ---
    # This creates a list of functions, each one responsible for creating an env with a unique port.
    env_fns = [lambda i=i: DarwinOPEnv(port=STARTING_PORT + i) for i in range(NUM_ENVIRONMENTS)]
    env = SubprocVecEnv(env_fns)

    # Agent Hyperparameters
    n_actions = env.action_space.shape[-1]
    action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.05 * np.ones(n_actions))

    # Create the Callback
    goal_callback = GoalCallback(check_freq=1000, goal_distance=5.0)

    # Check if a saved model exists
    if os.path.exists(MODEL_PATH):
        print(f"Found saved model at {MODEL_PATH}. Loading to continue training.")
        model = TD3.load(MODEL_PATH, env=env)
        new_logger = configure(TENSORBOARD_LOG_DIR, ["stdout", "csv", "tensorboard"])
        model.set_logger(new_logger)
    else:
        print("No saved model found. Creating new TD3 agent.")
        model = TD3(
            "MlpPolicy",
            env,
            action_noise=action_noise,
            verbose=1,
            tensorboard_log=TENSORBOARD_LOG_DIR,
            buffer_size=1_000_000,
            batch_size=256,
            gamma=0.99,
            learning_rate=1e-4,
            tau=0.005,
            learning_starts=10000,
            train_freq=(1, "step"), # Train after every step
            gradient_steps=-1 # As many gradient steps as transitions collected
        )

    try:
        # Note: log_interval is per environment. Total logs will be frequent.
        model.learn(
            total_timesteps=2_000_000,
            reset_num_timesteps=False,
            log_interval=10,
            callback=goal_callback
        )
    except KeyboardInterrupt:
        print("\nTraining interrupted by user.")
    finally:
        print("\nSaving final model...")
        model.save(MODEL_PATH)
        print(f"Model saved to {MODEL_PATH}")
        env.close()

