# TODOs:
# 1. Datenstruktur: Wie speichern wir die Partikel-Positionen und -Gewichte? DONE
# 2. Partikel initialisieren DONE
# 3. Partikel visualisieren DONE
# 4. Berechnen Sie die Framerate der ganzen Pipeline! DONE
# 5. Messkorrekturschritt <-- Lösung von ChatGPT mit Hilfe der scipy Distanz-Transformation umsetzen
# 6. Prädiktionsschritt 



import cv2
import numpy as np
import pfdemoparams
import time



cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Kann die Kamera nicht öffnen")
    exit()

frame_nr = 0

try:
    
    P = np.empty((pfdemoparams.N_particles, pfdemoparams.N_statedims+1))

    pop_initialized = False

    while True:
        start_time = time.time()

        ret, frame = cap.read()

        if not pop_initialized:
            P[:,0] = np.random.randint(0, frame.shape[1], size=pfdemoparams.N_particles) # initialise x-coords of particles
            P[:,1] = np.random.randint(0, frame.shape[0], size=pfdemoparams.N_particles) # initialise y-coords of particles
            P[:,2] = np.random.randint(-10, +10, size=pfdemoparams.N_particles) # initialise vx-component of particles
            P[:,3] = np.random.randint(-10, +10, size=pfdemoparams.N_particles) # initialise vy-component of particles
            P[:,4] = 1.0/pfdemoparams.N_particles     
            pop_initialized = True

        
        if not ret:
            print("Kann kein Frame lesen - Kameraausschluss?")
            break

        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([30,  30,  80])
        upper_yellow = np.array([65, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)        
        yellow_count = cv2.countNonZero(mask)

        if pop_initialized:
            for pnr in range(pfdemoparams.N_particles):
                x,y = int(P[pnr,0]), int(P[pnr,1])
                r = pfdemoparams.PARTICLE_VIS_RADIUS
                frame[y-r:y+r+1, x-r:x+r+1] = pfdemoparams.COLOR_PARTICLE
        
        frame_nr += 1        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        end_time = time.time()
        duration = end_time - start_time

        cv2.putText(frame, f'Frame: {frame_nr}, Yellow Pixels: {yellow_count}, FR={1/duration:.1f}',
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.imshow('Kamera-Feed', frame)
        cv2.imshow('Yellow Mask', mask)

finally:
    # free resources
    cap.release()
    cv2.destroyAllWindows()
