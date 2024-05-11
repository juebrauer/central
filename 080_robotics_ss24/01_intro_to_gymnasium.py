# Shows a single gymnasium RL framework environment.
#
# Install gymnasium with:
# pip install swig
# pip install gymnasium[box2d]
#
# If you have MESA driver problems, use the following command
# before starting this script:
#
# export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6
#
# See:
# https://stackoverflow.com/questions/72110384/libgl-error-mesa-loader-failed-to-open-iris
# https://stackoverflow.com/questions/71010343/cannot-load-swrast-and-iris-drivers-in-fedora-35/72200748#72200748


import gymnasium as gym
#env = gym.make("CartPole-v1", render_mode="human")
env = gym.make("LunarLander-v2", render_mode="human")
observation, info = env.reset(seed=42)
for _ in range(1000):
   action = env.action_space.sample()  # this is where you would insert your policy   
   print(action)
   observation, reward, terminated, truncated, info = env.step(action)   

   if terminated or truncated:
      print("Episode terminated/truncated")
      observation, info = env.reset()

env.close()