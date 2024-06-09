# TODOs
# 1. Lernverlauf kontrollieren <-- DONE
# 2. Landeversuche eines "gut trainierten" Agentens visualisieren! <-- DONE
# 3. Nutzt PyTorch eigentlich gerade meine GPU? <-- DONE
# 4. Code schneller machen (GPU-Unterstützung?) <-- DONE
# 5. ReplayBuffer: Ist der wirklich wichtig? Mal Code so umbauen, dass wir ihn ausschalten können <-- DONE
# 6. TargetNetwork: Unterscheidung zwischen dem eigentlichen Netzwerk und dem Netzwerk, das wir
#                   verwenden, um die Zielwerte zu bestimmen (zu approximieren) <-- DONE
# 7. Den Agenten mal "richtig lange" lernen lassen, um zu sehen, ob er dann fast immer landen kann
# 8. Optional: der Agent "sieht" nicht die 8 Zustandswerte, sondern "nur" das Bild
# 9. Optional: den hier entwickelten Code auf eine andere Umgebung anwenden!


import gymnasium as gym
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import random
from collections import deque
import pickle
import time

# Netzwerkdefinition
class DQN(nn.Module):
    def __init__(self, state_size, action_size):
        super(DQN, self).__init__()
        # Multi-Layer Perzeptron (MLPs): 8 Inputs --> 64 --> 64 --> 4 Outputs
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
EPSILON_START = 1.0
EPSILON_END = 0.01
EPSILON_DECAY = 200

# Erfahrungswiederholung
class ReplayBuffer:
    def __init__(self):
        self.buffer = deque(maxlen=MEMORY_SIZE)

    def add(self, state, action, reward, next_state, done):
        self.buffer.append((state, action, reward, next_state, done))        

    def sample(self, N):
        batch = random.sample(self.buffer, N)
        state, action, reward, next_state, done = map(np.array, zip(*batch))
        return state, action, reward, next_state, done

# Agent
class Agent:
    def __init__(self,
                 state_size,
                 action_size,
                 learning = True,
                 restore_agent_file = None,
                 use_replay_buffer = True,
                 use_target_network = False):        
        self.state_size = state_size
        self.action_size = action_size
        self.learning = learning
        self.use_replay_buffer = use_replay_buffer
        self.use_target_network = use_target_network

        if use_replay_buffer:
            self.batch_size = 64
        else:
            self.batch_size = 1

        if restore_agent_file != None:
            # start with an already trained neural network
            import pickle
            f = open(restore_agent_file, "rb")
            _ = pickle.load(f)
            self.network = pickle.load(f).to(device)
            f.close()
        else:
            # start with a fresh new neural network
            self.network = DQN(state_size, action_size).to(device)

            if self.use_target_network:
                self.target_network = DQN(state_size, action_size).to(device)
                self.target_network.load_state_dict(self.network.state_dict())

        self.epsilon = EPSILON_START        
        self.optimizer = optim.Adam(self.network.parameters(), lr=LEARNING_RATE)
        self.memory = ReplayBuffer()

        self.update_policy_step = 0

    def update_policy(self):
        if len(self.memory.buffer) < self.batch_size:
            return

        states, actions, rewards, next_states, dones = self.memory.sample( N=self.batch_size )
        states = torch.FloatTensor(states).to(device)
        actions = torch.LongTensor(actions).to(device)
        rewards = torch.FloatTensor(rewards).to(device)
        next_states = torch.FloatTensor(next_states).to(device)
        dones = torch.FloatTensor(dones).to(device)

        q_values = self.network(states).gather(1, actions.unsqueeze(1)).squeeze(1)

        # TODO! ACHTUNG!
        # Diese Implementierung nutzt nicht ein target-Network wie im Originalpaper
        # vorgeschlagen, um die Zielwerte (targets) zu berechnen!
        if self.use_target_network:
            next_q_values = self.target_network(next_states).max(1)[0]
        else:
            next_q_values = self.network(next_states).max(1)[0]
        target_q_values = rewards + GAMMA * next_q_values * (1 - dones)

        loss = nn.MSELoss()(q_values, target_q_values)

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        # Gewichte des Target-Netzwerks alle TARGET_UPDATE_FREQ Schritte aktualisieren
        self.update_policy_step += 1
        if self.update_policy_step % TARGET_UPDATE_FREQ == 0:
            self.target_network.load_state_dict(self.network.state_dict())


    def act(self, state):
        if self.learning and random.random() < self.epsilon:
            return random.randrange(self.action_size)
        else:            
            state = torch.FloatTensor(state).unsqueeze(0).to(device)
            q_values = self.network(state)
            return torch.argmax(q_values).item()

    def train_or_run(self, env, num_episodes):

        durations_per_episode = []
        returns_per_episode = []

        for episode in range(num_episodes):

            start_time = time.time()

            state, _ = env.reset()            
            return_episode = 0

            step_nr = 0
            while True:
                action = self.act(state)

                # observation, reward, terminated, truncated, info                
                next_state, reward, done, truncated, _ = env.step(action)             
                
                if self.learning:

                    if not self.use_replay_buffer:
                        # if we don't want to use the replay buffer,
                        # we delete old memories here
                        self.memory.buffer.clear()

                    self.memory.add(state, action, reward, next_state, done)
                    self.update_policy()

                state = next_state
                return_episode += reward

                if done or truncated:
                    break

                step_nr += 1

            # Update epsilon
            self.epsilon = max(self.epsilon - (EPSILON_START - EPSILON_END) / EPSILON_DECAY, EPSILON_END)
                        
            if self.learning:
                returns_per_episode.append( return_episode )
                if episode % 100 == 0:
                    f = open(f"learning/dqn_lunar_lander_rewards_per_episode_{episode:0>5}.pkl", "wb")
                    pickle.dump(returns_per_episode, f)
                    pickle.dump(self.network, f)
                    f.close()

            stop_time = time.time()
            duration = stop_time - start_time
            durations_per_episode.append( duration )
            mean_duration_per_episode = np.mean( durations_per_episode )
            est_duration_total_min = (mean_duration_per_episode * num_episodes)/60

            print(f"Episode {episode + 1}, " + \
                  f"Episode return: {return_episode:.2f}, " + \
                  f"Duration: {duration:.2f} [s], " + \
                  f"Mean duration: {mean_duration_per_episode:.2f} [s], " + \
                  f"Est. duration total: {est_duration_total_min:.0f} [min]"
                  )


print("-" * 30)
print("GPU-Infos:")
print(torch.cuda.current_device())
print(torch.cuda.get_device_name(torch.cuda.current_device()))

#device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
device = torch.device("cpu")
print(f"Ausführung des NN erfolgt auf: {device}")
print("-" * 30 + "\n")


# train or run?
train = True

# shall we use the replay buffer?
use_replay_buffer = True

# shall we use a target network (=an old variant of the network)
# to produce target Q values for the training?
TARGET_UPDATE_FREQ = 10
use_target_network = True

if train:
    # save computation time through deactivation of rendering
    env = gym.make('LunarLander-v2')
    fname = None
    
else:
    # activate rendering so that user can see what the agent does
    env = gym.make('LunarLander-v2', render_mode="human")    
    fname = "learning_run1/dqn_lunar_lander_rewards_per_episode_00990.pkl"

print(f"Observation / state space dim: {env.observation_space.shape[0]}")
print(f"Action space dim: {env.action_space.n}")
print(f"Learning: {train}")
print(f"Restore agent file: {fname}")
print(f"Use replay buffer: {use_replay_buffer}")
print(f"Use target network: {use_target_network}")
print()

agent = Agent(env.observation_space.shape[0],
              env.action_space.n,
              learning = train,
              restore_agent_file = fname,
              use_replay_buffer = use_replay_buffer,
              use_target_network = use_target_network )
agent.train_or_run(env, 10000)  # Anzahl der Episoden anpassen
env.close()


