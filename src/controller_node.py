#!/usr/bin/python3.8

import argparse
from stable_baselines3 import PPO
import rospy
import numpy as np
from drone_env_PPO import DroneEnv




def main():
    """
    function to handle ros parameters adn load environment and model
    """
    rospy.init_node('argument_handler_node')

    args = parse_arguments()
    
    # Fetch parameters from the ROS parameter server, prioritize command-line arguments if provided
    drone = args.drone if args.execution else rospy.get_param('~drone', 'drone1')    # Default to 'drone1' if not set
    execution = args.execution if args.execution else rospy.get_param('~execution', 'SIM')    # Default to 'SIM' if not set
    controller = args.controller if args.controller else rospy.get_param('~controller', 'PID')  # Default to 'PID' if not set

    rospy.loginfo(f"Drone: {drone}")
    rospy.loginfo(f"Execution mode: {execution}")
    rospy.loginfo(f"Controller: {controller}")

    env = DroneEnv(drone=drone, execution=execution, controlller=controller, training=False)

    model = PPO.load("/home/srijan1804/Colosseum/ros/src/drl_new/scripts/modules/DRL/checkpoints_xyz_disc7/checkpoints/drlppo_xyz_disc_7actions_800000_steps.zip")
    print("Model loaded!")

    return model, env



def parse_arguments():
    """
    function to handle commandline arguments
    """
    parser = argparse.ArgumentParser(description="Run the controller node with specified execution mode and controller type.")
    parser.add_argument('--drone', type=str, default=None, help="Drone namespace-> 'drone1', 'drone2' or similar")
    parser.add_argument('--execution', type=str, default=None, help="Execution mode-> 'SIM' or 'DEPLOY'")
    parser.add_argument('--controller', type=str, default=None, help="Controller type-> 'PID' or 'DRL'")
    args = parser.parse_args()
    return args



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
    model, env = main()
    try:
        print("Running Controller!")
        mean_reward, std_reward = run_inference(model=model, env=env, num_episodes=1)
        print(f"DRL Controller -> Mean Reward: {mean_reward} ± {std_reward}")
    except rospy.ROSInterruptException:
        pass
