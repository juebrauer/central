import numpy as np
import random

class GridWorld:
    def __init__(self, height, width, start_pos, goal_pos):
        
        self.height = height
        self.width = width        

        self.start_pos = start_pos
        self.goal_pos = goal_pos

        self.state = start_pos
    
    def reset(self):

        always_same_start_position = True

        if always_same_start_position:
            self.state = self.start_pos
        else:
            rndy = np.random.randint(low=0, high=self.height)
            rndx = np.random.randint(low=0, high=self.width)
            self.state = (rndy, rndx)

        return self.state
    
    def step(self, action):
        y, x = self.state
        if action == 0:   # hoch
            y = max(0, y-1)
        elif action == 1: # runter
            y = min(self.height - 1, y + 1)
        elif action == 2: # links
            x = max(0, x - 1)
        elif action == 3: # rechts
            x = min(self.width - 1, x + 1)
        
        self.state = (y, x)
        reward = -1 # Standardbelohnung
        done = self.state == self.goal_pos
        
        if done:
            reward = 0 # Belohnung für das Erreichen des Ziels
        return self.state, reward, done

# Q-Learning
def q_learning(env, episodes, alpha, gamma, epsilon):
    q_table = np.zeros((4, env.height, env.width))
    
    for episode in range(episodes):
        state = env.reset()
        done = False
        
        while not done:
            if random.uniform(0, 1) < epsilon:
                action = random.randint(0, 3) # Exploration
            else:
                action = np.argmax(q_table[:, state[0], state[1]]) # Exploitation
            
            next_state, reward, done = env.step(action)
            old_value = q_table[action, state[0], state[1]] # Q(s,a)
            next_max = np.max(q_table[:, next_state[0], next_state[1]]) 
            
            # Q-Wert-Update
            q_table[action, state[0], state[1]] = \
                old_value + alpha * (reward + gamma * next_max - old_value)
            
            state = next_state
            
    return q_table

# Beste Aktionen visualisieren
def best_actions(q_table):
    print(q_table.shape)
    action_symbols = ['^', 'v', '<', '>']
    print(action_symbols)
    best_actions_map = np.argmax(q_table, axis=0)
    print(best_actions_map)
    print(best_actions_map.shape)

    for y in range(best_actions_map.shape[0]):
        row = ''
        for x in range(best_actions_map.shape[1]):
            row += action_symbols[best_actions_map[y, x]] + ' '
        print(row)

env = GridWorld(height=9, width=5, start_pos=(0, 0), goal_pos=(8, 4))
#env = GridWorld(height=9, width=5, start_pos=(0,0), goal_pos=(4, 2))

# Q-Learning Parameter
episodes = 1000
alpha = 0.1
gamma = 0.95
epsilon = 0.1

# Training
q_table = q_learning(env, episodes, alpha, gamma, epsilon)

# Beste Aktionen anzeigen
print("\nVisualisierung der besten Aktionen in jedem Zustand:")
best_actions(q_table)
