import sys

def main():
    # ... Ihr Programmcode ...
    
    irgendein_fehler_aufgetreten = True

    if irgendein_fehler_aufgetreten:
        print("Ups!")
        print("Da ist wohl was schief gelaufen!")
        sys.exit(42)  # Beendet das Programm mit einem Fehlercode von 42
    else:
        sys.exit(0)  # Beendet das Programm erfolgreich mit einem Exit-Code von 0

        
main()