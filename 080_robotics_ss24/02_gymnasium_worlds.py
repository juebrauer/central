# Demo that shows some of the environments
# of the RL framework 'gymnnasium'
#
# Gymnasium is a maintained fork of OpenAI’s Gym library.
# Why this fork? Read: https://farama.org/Announcing-The-Farama-Foundation
# https://farama.org/
# https://gymnasium.farama.org/
# https://www.gymlibrary.dev/
#
# Note:
# If you have "MESA" (Open Source implementation of OpenGL) related problems,
# use the following command before starting this script:
#
# export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6
#
# See:
# https://stackoverflow.com/questions/72110384/libgl-error-mesa-loader-failed-to-open-iris
# https://stackoverflow.com/questions/71010343/cannot-load-swrast-and-iris-drivers-in-fedora-35/72200748#72200748
#
# The problem with the MESA seems to occur in conda/miniconda virtual environments.

import gymnasium as gym
import logging

# configure where, what and how to log
logging.basicConfig(filename='logfile.txt',
                    filemode='w',
                    level=logging.DEBUG,
                    format='%(message)s'
                    #format='%(asctime)s:%(levelname)s:%(message)s'
                   )


# See:
# https://stackoverflow.com/questions/77438425/mujoco-structs-mjdata-object-has-no-attribute-solver-iter
# pip install mujoco==2.3.0
# pip install mujoco-py
# env_name = "Humanoid-v4"

envs = ["CartPole-v1", "LunarLander-v2", "BipedalWalker-v3"]
#envs = ["CartPole-v1"]

logging.info("Available environments:")
for e in gym.envs.registry.keys():
   logging.info(f"\t {e}")

for env_name in envs:

   logging.info("")
   logging.info("-"*10)
   logging.info(f"Generating environment '{env_name}'")
   env = gym.make(env_name, render_mode="human") # should display the environment
   #env = gym.make(env_name) # doesn't display the environment

   logging.info(f"observation space shape: {env.observation_space.shape}")
   logging.info(f"observation space: {env.observation_space}")

   logging.info(f"action space shape: {env.action_space.shape}")
   logging.info(f"action space: {env.action_space}")

   logging.info(f"reward range: {env.reward_range}")

   logging.info(f"spec: {env.spec}")

   logging.info(f"metadata: {env.metadata}")

   logging.info("Example actions:")   
   for _ in range(10):
      logging.info( env.action_space.sample() )

   observation, info = env.reset(seed=42)

   observations = []
   rewards = []
   terminateds = []
   truncateds = []
   infos = []
   actions = []
   
   for t in range(100):
      action = env.action_space.sample()  # this is where you would insert your policy
      observation, reward, terminated, truncated, info = env.step(action)

      if t<=3:
         observations.append( observation )
         rewards.append( reward )
         terminateds.append( terminated )
         truncateds.append( truncated )
         infos.append( info )
         actions.append( action )

      if t==3:
         logging.info(f"observations: {observations}")
         logging.info(f"rewards: {rewards}")
         logging.info(f"terminateds: {terminateds}")
         logging.info(f"truncateds: {truncateds}")
         logging.info(f"infos: {infos}")
         logging.info(f"actions: {actions}")

      if terminated or truncated:      
         observation, info = env.reset()

   env.close()