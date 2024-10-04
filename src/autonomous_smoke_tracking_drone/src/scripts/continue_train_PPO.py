#!/usr/bin/python3.8

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
import numpy as np
from drone_env_PPO import DroneEnv
from cnn_policy import CnnPolicy



# ------------------------------------ HYPERPARAMETERS ------------------------------------#
N_STEPS = 2048            # Number of steps to run for each environment per update (default: 2048)
BATCH_SIZE = 128          # Minibatch size (default: 64)
N_EPOCHS = 10             # Number of epochs when optimizing the surrogate loss (default: 10)
MODEL_SAVE_FREQ = 100000  # Save the model every no of step
ADDITIONAL_TIMESTEPS = 400000
# ------------------------------------ HYPERPARAMETERS ------------------------------------#



# ----------------------------------- LOGGING DIRECTORY -----------------------------------#
MAIN_DIR = "/home/srijan1804/Colosseum/ros/src/drl_new/scripts/modules/DRL/checkpoints_xyz_disc7/"
LOG_DIR = MAIN_DIR + "tb_log/"
MODEL_SAVE_DIR = MAIN_DIR + "models/"
BEST_MODEL_SAVE_DIR = MAIN_DIR + "best_model/"
CHECKPOINT_DIR = MAIN_DIR + "checkpoints/"
# ----------------------------------- LOGGING DIRECTORY -----------------------------------#

EXECUTION = 'SIM'



droneEnv = DroneEnv(execution=EXECUTION, training=True)

model_path = CHECKPOINT_DIR + 'drlppo_xyz_disc_7actions_400000_steps_400000_steps.zip'
model = PPO.load(model_path, env=droneEnv)

# CheckpointCallback
checkpoint_callback = CheckpointCallback(
    save_freq=MODEL_SAVE_FREQ, save_path=CHECKPOINT_DIR,
    name_prefix="drlppo_xyz_disc_7actions_400000_steps_400000_steps")


model.learn(total_timesteps=ADDITIONAL_TIMESTEPS, callback=checkpoint_callback)

model.save(BEST_MODEL_SAVE_DIR + "drlppo_xyz_disc_7actions_400000_steps_400000_steps_400000_steps.zip")
print("Model Saved!")
