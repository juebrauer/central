# Show how to wait in a non-blocking fashion
# for a key from user and read it out

import pygame
import sys

# Initialisiere Pygame
pygame.init()

# Fenstergröße
screen_width = 640
screen_height = 480

# Farbdefinitionen
black = (0, 0, 0)
white = (255, 255, 255)

# Erstelle das Pygame-Fenster
screen = pygame.display.set_mode((screen_width, screen_height))

# Titel des Fensters
pygame.display.set_caption("Tastatureingabe anzeigen")

# Initialisiere die Schriftart
font = pygame.font.Font(None, 72)

# Variable für den zuletzt gedrückten Buchstaben
last_key = ""

counter = 0

# Hauptschleife
running = True
while running:
    eventlist = pygame.event.get()
    #print(len(eventlist), "eventlist=",eventlist)
    for event in eventlist:
        if event.type == pygame.QUIT:
            running = False
        # Überprüfe auf Tastendruck-Ereignisse
        elif event.type == pygame.KEYDOWN:
            print("KEYDOWN")
            # Aktualisiere den zuletzt gedrückten Buchstaben
            # pygame.key.name(event.key) gibt den Namen der Taste als String zurück
            last_key = pygame.key.name(event.key)
        elif event.type == pygame.KEYUP:
            print("KEYUP")

    # Fülle den Bildschirm mit Schwarz
    screen.fill(black)

    # Erstelle ein Text-Surface mit dem zuletzt gedrückten Buchstaben
    text_surface = font.render(f"{counter}:{last_key}", True, white)

    # Berechne die Position des Textes, um ihn zentriert auf dem Bildschirm anzuzeigen
    text_rect = text_surface.get_rect(center=(screen_width/2, screen_height/2))

    # Zeichne den Text auf den Bildschirm
    screen.blit(text_surface, text_rect)

    # Aktualisiere den Bildschirm
    pygame.display.flip()

    counter += 1

# Beende Pygame, wenn die Schleife beendet wird
pygame.quit()
sys.exit()
