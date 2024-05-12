import gymnasium as gym
import numpy as np

# Parameter für Q-Learning
alpha = 0.1  # Lernrate
gamma = 0.99  # Diskontierungsfaktor
epsilon = 0.1  # Wahrscheinlichkeit der Exploration

def discretize(obs):
    # Anpassung der kontinuierlichen Beobachtungen an diskrete Zustände
    # Diese Funktion muss basierend auf den Eigenschaften des Zustandsraums angepasst werden
    # (x, y, x_dot, y_dot, angle, angular_vel, left_leg, right_leg) =
    # (29, ...)
    # (2.9, ...)
    # (2, ...)
    return tuple((obs / np.array([0.1, 0.1, 0.1, 0.1, 1, 1, 1, 1])).astype(int))

def choose_action(state):
    # Epsilon-greedy Strategie zur Aktionenauswahl
    if np.random.random() < epsilon:
        return env.action_space.sample()  # Zufällige Aktion
    else:
        return np.argmax(q_table[state])  # Beste Aktion basierend auf Q-Tabelle

# Erstellung der Umgebung
env = gym.make("LunarLander-v2", render_mode="human")
n_actions = env.action_space.n

# Q-Tabelle Initialisierung
print("Q-Tabelle Initialisierung START")
q_table = {}
for x in range(-10, 11):
    print("x=",x)
    for y in range(-10, 11):
        print("y=",y)
        for x_dot in range(-10, 11):
            for y_dot in range(-10, 11):
                for angle in range(-10, 11):
                    for angular_vel in range(-10, 11):
                        for left_leg in range(2):
                            for right_leg in range(2):
                                state = (x, y, x_dot, y_dot, angle, angular_vel, left_leg, right_leg)
                                q_table[state] = np.zeros(n_actions)
print("Q-Tabelle Initialisierung ENDE")

# Trainingszyklus
num_episodes = 2000  # Anzahl der Episoden
for _ in range(num_episodes):
    if num_episodes % 10 == 0:
        print(f"num_episodes={num_episodes}")

    observation, info = env.reset(seed=42)
    state = discretize(observation)

    while True:
        action = choose_action(state)
        new_observation, reward, terminated, truncated, info = env.step(action)
        new_state = discretize(new_observation)

        # Q-Wert aktualisieren
        best_next_action = np.argmax(q_table[new_state])
        td_target = reward + gamma * q_table[new_state][best_next_action]
        td_error = td_target - q_table[state][action]
        q_table[state][action] += alpha * td_error

        state = new_state

        if terminated or truncated:
            break

env.close()
