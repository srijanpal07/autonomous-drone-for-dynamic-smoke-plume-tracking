#!/usr/bin/python3.8

from stable_baselines3 import PPO
import rospy
import numpy as np
from drone_env_PPO import DroneEnv

EXECUTION = 'SIM' # 'SIM' or 'DEPLOY'
ENV = DroneEnv(execution=EXECUTION, training=False)

MODEL = PPO.load("/home/srijan1804/Colosseum/ros/src/drl_new/scripts/modules/DRL/checkpoints_xyz_disc7/checkpoints/drlppo_xyz_disc_7actions_800000_steps.zip")
print("Model loaded!")


def run_inference(model, env, num_episodes=1):
    """
    function to run inference
    """
    total_rewards = []
    for episode in range(num_episodes):
        obs = env.reset()
        done = False
        episode_reward = 0
        while not done:
            action, _ = model.predict(obs)
            obs, reward, done, _ = env.step(action)
            episode_reward += reward

        total_rewards.append(episode_reward)
        print(f"Episode {episode + 1}: Total Reward: {episode_reward}")

    mean_rwrd = np.mean(total_rewards)
    std_rwrd = np.std(total_rewards)

    return mean_rwrd, std_rwrd




if __name__ == '__main__':
    try:
        print("Running Inference!")
        mean_reward, std_reward = run_inference(model=MODEL, env=ENV, num_episodes=1)
        print(f"Mean Reward: {mean_reward} ± {std_reward}")
    except rospy.ROSInterruptException:
        pass
