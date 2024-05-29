Ich habe ein 2D NumPy Array P. In der ersten Spalte steht die x-Koordinate eines Partikels und in der zweiten Spalte die y-Koordinate eines Partikels. Ich habe zudem eine Maske mask eines Bildes (jeweils auch ein 2D NumPy-Array mit Breite B und Höhe H), bei der für jedes Pixel im Bild in der Maske steht, ob das Pixel gelb ist oder nicht. Meine Aufgabe: jetzt muss ich für jedes Partikel im Array P feststellen, was ist das nächste gelbe Pixel, d.h. wo steht in der Maske am nächsten eine 1? (Maskenwert 0=kein gelbes Pixel, Maskenwert 1=gelbes Pixel)


Um das nächste gelbe Pixel für jedes Partikel im Array PP zu finden, können wir eine effiziente Methode nutzen, die auf der Berechnung der Distanzen von jedem Partikel zu jedem gelben Pixel basiert. Diese Methode wird jedoch mit zunehmender Größe des Arrays und der Anzahl der gelben Pixel zunehmend rechenaufwendig. Eine Alternative ist die Nutzung eines Distanztransformation-Algorithmus, der die Entfernung jedes Pixels in der Maske zu dem nächsten gelben Pixel berechnet.

Hier sind die Schritte, die du durchführen kannst:

    Berechne die Distanztransformation der Maske: Wandele die Maske so um, dass die Werte 0 zu 1 werden und die Werte 1 zu 0. Anschließend führe eine Distanztransformation durch, die für jedes Pixel den Abstand zum nächsten gelben Pixel (ursprünglich 1 in der Maske) berechnet.

    Bestimme das nächste gelbe Pixel für jedes Partikel: Für jedes Partikel in PP suche das nächstgelegene Pixel, das den kürzesten Abstand im Ergebnis der Distanztransformation hat.

Lass uns dies in Python implementieren. Ich werde den Code schrittweise erklären, und du kannst ihn entsprechend deinen Daten anpassen.

Zuerst müssen wir sicherstellen, dass wir die notwendigen Bibliotheken installiert haben. Du brauchst numpy und scipy für die Distanztransformation:

python



import numpy as np
from scipy.ndimage import distance_transform_edt

# Angenommen mask ist dein 2D Array für die Maske
# und P ist dein 2D Array der Partikelkoordinaten

# Schritt 1: Berechne die Distanztransformation der Maske
# Wir invertieren die Maske, da distance_transform_edt die Distanz zu den Nullen berechnet
inverted_mask = 1 - mask
distances = distance_transform_edt(inverted_mask)

# Schritt 2: Bestimme das nächste gelbe Pixel für jedes Partikel
closest_yellow_pixels = []
for x, y in P:
    # Findet die Koordinaten des nächsten gelben Pixels (mit der kleinsten Distanz)
    local_min = np.unravel_index(np.argmin(distances[int(x)-1:int(x)+2, int(y)-1:int(y)+2], axis=None), (3,3))
    closest_yellow_pixels.append((local_min[0] + int(x) - 1, local_min[1] + int(y) - 1))

closest_yellow_pixels = np.array(closest_yellow_pixels)