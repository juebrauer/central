# TODOs:
# 1. Datenstruktur: Wie speichern wir die Partikel-Positionen und -Gewichte? DONE
# 2. Partikel initialisieren DONE
# 3. Partikel visualisieren DONE
# 4. Berechnen Sie die Framerate der ganzen Pipeline! DONE
# 5. Messkorrekturschritt
#    5.1 Rauschen im Messkorrekturschritt reduzieren DONE
#    5.2 Einen Teil der Partikelpopulation in jedem Aktualisierungsschritt des PF wieder zufällig neu verteilen  DONE
#    5.3 Den Hyperparameter PARTICLE_FOLLOW_MEASUREMENT_SPEED anpassen DONE
# 6. Prädiktionsschritt <--



import cv2
import numpy as np
import pfdemoparams
import time
from scipy.spatial import KDTree
from scipy.stats import gaussian_kde

def list_available_cameras():
    available_cameras = []
    for camera_id in range(10):
        print(f"Versuche Zugriff auf Kamera Nr. {camera_id} ...", end=" ")
        cap = cv2.VideoCapture(camera_id)
        if cap.isOpened():
            print("ERFOLG!")
            available_cameras.append(camera_id)
            cap.release()
        else:
            print("FEHLGESCHLAGEN!")
    return available_cameras


def compute_pdf(P, frame_height, frame_width, grid_size=100):

    y_coords = P[:, 0]
    x_coords = P[:, 1]

    # Erstelle die 2D-Daten für den KDE
    data = np.vstack([x_coords, y_coords])

    # Erzeuge einen KDE
    kde = gaussian_kde(data)

    # Erstelle ein Raster, um die Wahrscheinlichkeitsdichte zu evaluieren
    xmin, xmax = 0, frame_width
    ymin, ymax = 0, frame_height

    X, Y = np.mgrid[xmin:xmax:complex(0, grid_size), ymin:ymax:complex(0, grid_size)]
    positions = np.vstack([X.ravel(), Y.ravel()])
    Z = np.reshape(kde(positions).T, X.shape)
    
    return Z


def generate_pdf_visualtion(Z):

    # Normiere Z für die Visualisierung
    Z = (Z - Z.min()) / (Z.max() - Z.min())
    Z = (Z * 255).astype(np.uint8)
    Z = Z.T

    # Erzeuge ein RGB-Bild
    Z_color = cv2.applyColorMap(Z, cv2.COLORMAP_COOL)

    return Z_color

    
available_cameras = list_available_cameras()
print("-" * 40)
print("Verfügbare Kameras:", available_cameras)
print("-" * 40 + "\n")

cap = cv2.VideoCapture(2)
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
            # Initialize a random (y,x,vy,vx) coordinate for each the N_particles particles
            P[:,0] = np.random.randint(0, frame.shape[0], size=pfdemoparams.N_particles) # initialise y-coords of particles
            P[:,1] = np.random.randint(0, frame.shape[1], size=pfdemoparams.N_particles) # initialise x-coords of particles
            P[:,2] = np.random.randint(-10, +10, size=pfdemoparams.N_particles) # initialise vy-component of particles
            P[:,3] = np.random.randint(-10, +10, size=pfdemoparams.N_particles) # initialise vx-component of particles
            P[:,4] = 1.0/pfdemoparams.N_particles     
            pop_initialized = True
            print(P.dtype)

        
        if not ret:
            print("Kann kein Frame lesen - Kameraausschluss?")
            break

        
        # Detect yellow pixels
        # The result is a mask: 255 if it is a yellow pixel, 0 else
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([30,  30,  80])
        upper_yellow = np.array([65, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)        
        yellow_count = cv2.countNonZero(mask)
        
        if pop_initialized:

            # 1.
            # Visualize each particle by a square
            for pnr in range(pfdemoparams.N_particles):
                y,x,vy,vx = int(P[pnr,0]), int(P[pnr,1]), int(P[pnr,2]), int(P[pnr,3])                
                r = pfdemoparams.PARTICLE_VIS_RADIUS
                frame[y-r:y+r+1, x-r:x+r+1] = pfdemoparams.COLOR_PARTICLE
                cv2.line(frame, (x,y), (x+vx,y+vy), (0,255,255))


            # 2.
            # Für jedes Partikel das nächste gelbe Pixel (=Messung) bestimmen

            # Finde alle gelben Pixel (Wert 255 in der Maske)
            # Das Result ist ein NumPy-Array mit der Shape (Anzahl der gelben Pixel, 2)
            # Achtung!
            # In dem NumPy-Array gelbe_pixel stehen
            # ... in der 1. Spalte y-Koordinaten
            # ... in der 2. Spalte x-Koordinaten
            gelbe_pixel = np.argwhere(mask == 255)

            if len(gelbe_pixel) > 0:
                       
                # Liste zur Speicherung der nächsten gelben Pixel für jedes Partikel
                naechste_gelbe_pixel = []

                # Erzeuge einen k-d-Tree aus den gelben Pixeln
                kd_tree = KDTree(gelbe_pixel)

                # Liste zur Speicherung der nächsten gelben Pixel für jedes Partikel
                naechste_gelbe_pixel = []

                # Iteriere über jedes Partikel
                for y_p, x_p, _, _, _ in P:
                    # Finde das nächstgelegene gelbe Pixel mithilfe des k-d-Trees
                    dist, index = kd_tree.query((y_p, x_p))
                    naechstes_pixel = tuple(gelbe_pixel[index])                    
                    
                    # Füge das nächstgelegene gelbe Pixel zur Liste hinzu
                    naechste_gelbe_pixel.append(naechstes_pixel)

                # Optional: die Liste in ein NumPy-Array umwandeln
                naechste_gelbe_pixel = np.array(naechste_gelbe_pixel)
                #print(naechste_gelbe_pixel[:30])
                            

                # 3.
                # Jedes Partikel "ein Stückchen" in Richtung seiner bestätigenden
                # nächsten Messung bewegen   
                for pnr in range(pfdemoparams.N_particles):
                    y,x = int(P[pnr,0]), int(P[pnr,1])
                    ny,nx = naechste_gelbe_pixel[pnr]

                    # Partikel ein Stück in Richtung nächster Messung bewegen
                    dirx, diry = nx-x, ny-y
                    neu_x = x + pfdemoparams.PARTICLE_FOLLOW_MEASUREMENT_SPEED * dirx
                    neu_y = y + pfdemoparams.PARTICLE_FOLLOW_MEASUREMENT_SPEED * diry

                    # Rauschen drauf addieren
                    rndx, rndy = np.random.normal(loc=0, scale=pfdemoparams.NOISE_MEASUREMENT_CORRECTION_STEP, size=2)
                    neu_x += rndx
                    neu_y += rndy

                    # Neue Partikelposition speichern                    
                    P[pnr,0] = neu_y
                    P[pnr,1] = neu_x

                    # Die geschätzte Objektgeschwindigkeit aktualisieren
                    P[pnr,2] = 0.5*diry + 0.5*P[pnr,2]
                    P[pnr,3] = 0.5*dirx + 0.5*P[pnr,3]



                # 4.
                # Einen Teil der Population zufällig "in der Nähe"
                # von Messungen verteilen
                # P : Population
                # mask : Maske
                # 1. Teilproblem: Wie kann ich 5% zufällig von den Partikeln auswählen?
                # 2. Teilproblem: Wie kann ich aus der Maske eines der gelben Pixel zufällig auswählen?

                # Anzahl der auszuwählenden Indizes aus der Partikelpopulation
                num_to_select = int(0.05 * pfdemoparams.N_particles)

                # Zufällige Auswahl der Indizes
                selected_particles_indices = np.random.choice(np.arange(pfdemoparams.N_particles), size=num_to_select, replace=False)

                # Für alle Partikel, die neu gesetzt werden sollen ... 
                anz_gelbe_pixel = len(gelbe_pixel)               
                for pnr in selected_particles_indices:

                    # Wähle eines der anz_gelbe_pixel vielen gelben Pixel zufällig aus
                    rnd_pixel_index = np.random.randint(low=0, high=anz_gelbe_pixel)

                    # Hole die mask-2D-Koordinate dieses gelben Pixels
                    gelbe_pixel_coord_in_mask = gelbe_pixel[rnd_pixel_index]

                    # setze das Partikel auf diese Messung = gelbes Pixel
                    P[pnr,0] = gelbe_pixel_coord_in_mask[0] # setze y-Koordinate vom Partikel neu auf gelbes Pixel
                    P[pnr,1] = gelbe_pixel_coord_in_mask[1] # setze x-Koordinate vom Partikel neu auf gelbes Pixel

            # 5. Prädiktionsschritt
            P[pnr,0] += P[pnr, 2] # y=y+vy
            P[pnr,1] += P[pnr, 3] # x=x+vx
                    
        frame_nr += 1      

        # If user presses q, we quit this demo  
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        end_time = time.time()
        duration = end_time - start_time

        cv2.putText(frame, f'Frame: {frame_nr}, Yellow Pixels: {yellow_count}, FR={1/duration:.1f}',
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.imshow('Kamera-Feed', frame)
        cv2.imshow('Yellow Mask', mask)

        # Bei gegebener Partikelpopulation P
        # berechne auf einem Raster Z von (100,100) Punkten die Dichte
        # dann mappe diese (100,100) Werte auf Farbwerte (pdf_visu)
        # und zeige pdf_visu an
        if frame_nr % 3 == 0:
            Z = compute_pdf(P, frame_height=frame.shape[0], frame_width=frame.shape[1])
            pdf_visu = generate_pdf_visualtion(Z)
            cv2.imshow('PDF', pdf_visu)


finally:
    # free resources
    cap.release()
    cv2.destroyAllWindows()
