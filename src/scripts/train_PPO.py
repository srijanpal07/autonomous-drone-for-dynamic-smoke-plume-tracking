#!/usr/bin/python3.8

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
import numpy as np
from drone_env_PPO import DroneEnv
from cnn_policy import CnnPolicy

EXECUTION = 'SIM' # 'SIM' or 'DEPLOY'

# ------------------------------------ HYPERPARAMETERS ------------------------------------#
N_STEPS = 2048             # Number of steps to run for each environment per update (default: 2048)
BATCH_SIZE = 256           # Minibatch size (default: 64)
N_EPOCHS = 10              # Number of epochs when optimizing the surrogate loss (default: 10)
MODEL_SAVE_FREQ = 400000   # Save the model every no of step
MAIN_DIR = "/home/srijan1804/Colosseum/ros/src/drl_new_test/scripts/modules/DRL/checkpoints_xyz_disc7/"
LOG_DIR = MAIN_DIR + "tb_log/"
MODEL_SAVE_DIR = MAIN_DIR + "models/"
LAST_MODEL_SAVE_DIR = MAIN_DIR + "last_model/"
CHECKPOINT_DIR = MAIN_DIR + "checkpoints/"
TIMESTEPS = 400000
# ------------------------------------ HYPERPARAMETERS ------------------------------------#


droneEnv = DroneEnv(execution=EXECUTION, training=True)

# CheckpointCallback
checkpoint_callback = CheckpointCallback(
    save_freq=MODEL_SAVE_FREQ, save_path=CHECKPOINT_DIR,
    name_prefix="drlppo_xyz_disc_7actions_test")


# Model Initialization
model = PPO(CnnPolicy, droneEnv, verbose=1, n_steps=N_STEPS, batch_size=BATCH_SIZE,
            n_epochs=N_EPOCHS, tensorboard_log=LOG_DIR)


# Training the model
model.learn(total_timesteps=TIMESTEPS, callback=checkpoint_callback)


# Saving model
model.save(MODEL_SAVE_DIR + "drlppo_xyz_disc_7actions_test_700000_steps")
print("Model Saved!")
