import gymnasium as gym
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import random
from collections import deque

# Netzwerkdefinition
class DQN(nn.Module):
    def __init__(self, state_size, action_size):
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(state_size, 64)
        self.fc2 = nn.Linear(64, 64)
        self.fc3 = nn.Linear(64, action_size)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        return self.fc3(x)

# Hyperparameter
GAMMA = 0.99
LEARNING_RATE = 0.001
MEMORY_SIZE = 100000
BATCH_SIZE = 64
EPSILON_START = 1.0
EPSILON_END = 0.01
EPSILON_DECAY = 200

# Erfahrungswiederholung
class ReplayBuffer:
    def __init__(self):
        self.buffer = deque(maxlen=MEMORY_SIZE)

    def add(self, state, action, reward, next_state, done):
        self.buffer.append((state, action, reward, next_state, done))

    def sample(self):
        batch = random.sample(self.buffer, BATCH_SIZE)
        state, action, reward, next_state, done = map(np.array, zip(*batch))
        return state, action, reward, next_state, done

# Agent
class Agent:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.epsilon = EPSILON_START
        self.network = DQN(state_size, action_size)
        self.optimizer = optim.Adam(self.network.parameters(), lr=LEARNING_RATE)
        self.memory = ReplayBuffer()

    def update_policy(self):
        if len(self.memory.buffer) < BATCH_SIZE:
            return

        states, actions, rewards, next_states, dones = self.memory.sample()
        states = torch.FloatTensor(states)
        actions = torch.LongTensor(actions)
        rewards = torch.FloatTensor(rewards)
        next_states = torch.FloatTensor(next_states)
        dones = torch.FloatTensor(dones)

        q_values = self.network(states).gather(1, actions.unsqueeze(1)).squeeze(1)
        next_q_values = self.network(next_states).max(1)[0]
        target_q_values = rewards + GAMMA * next_q_values * (1 - dones)

        loss = nn.MSELoss()(q_values, target_q_values)

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

    def act(self, state):
        if random.random() < self.epsilon:
            return random.randrange(self.action_size)
        else:
            state = torch.FloatTensor(state).unsqueeze(0)
            q_values = self.network(state)
            return torch.argmax(q_values).item()

    def train(self, env, num_episodes):
        for episode in range(num_episodes):
            state = env.reset()
            total_reward = 0

            while True:
                action = self.act(state)
                next_state, reward, done, _ = env.step(action)
                self.memory.add(state, action, reward, next_state, done)
                self.update_policy()

                state = next_state
                total_reward += reward

                if done:
                    break

            # Update epsilon
            self.epsilon = max(self.epsilon - (EPSILON_START - EPSILON_END) / EPSILON_DECAY, EPSILON_END)
            print(f"Episode {episode + 1}, Total Reward: {total_reward}")

# Haupttrainingsschleife
env = gym.make('LunarLander-v2', render_mode='human')
agent = Agent(env.observation_space.shape[0], env.action_space.n)
agent.train(env, 1000)  # Anzahl der Episoden anpassen
env.close()
