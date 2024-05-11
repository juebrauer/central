# Let the user control the agent in one of the environments
#
# If you have MESA related problems:
# export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6

import gymnasium as gym
import pygame
import time

# Erstelle die Gym-Umgebung
#env_name = "CartPole-v1"
env_name = "LunarLander-v2"

env = gym.make(env_name, render_mode="human")
observation, info = env.reset(seed=42)

screen = pygame.display.get_surface()
# Initialisiere die Schriftart
font = pygame.font.Font(None, 36)  # None nutzt die Standard-Schriftart, 36 ist die Größe


# Counter für die Aktionen
action = 0
counter = 0

last_key = None

running = True
while running:
    eventlist = pygame.event.get()
    for event in eventlist:
        if event.type == pygame.QUIT:
            running = False
        # Überprüfe auf Tastendruck-Ereignisse
        elif event.type == pygame.KEYDOWN:            
            last_key = pygame.key.name(event.key)
        
    if env_name=="CartPole-v1":
        match last_key:
            case "left":
                action = 0 # left
            case "right":
                action = 1 # right
            case _ :
                action = 0
    
    if env_name=="LunarLander-v2":
        match last_key:            
            case "right":
                action = 1 # fire control nozzle right
            case "down" :
                action = 2 # fire control nozzle down
            case "left" :
                action = 3 # fire control nozzle left
            case "up":
                action = 0 # do nothing

    observation, reward, terminated, truncated, info = env.step(action)
    counter += 1

    # reset action for LunarLander environment
    if env_name=="LunarLander-v2":
        last_key = "None"
        action = 0

    if terminated or truncated:
        observation, info = env.reset()
        counter = 0

        
    text = font.render(f"Counter: {counter} / action: {action}", True, (0,255,0))    
    screen.blit(text, (10, 10))
    pygame.display.flip()

    time.sleep(0.2)

# Schließe die Gym-Umgebung und Pygame, wenn die Schleife beendet wird
env.close()
pygame.quit()
