// package ist ein Schlüsselwort in Go.
// Es definiert zu welchem Paket diese Datei gehört.
// Es kann übrigens nur ein Paket pro Verzeichnis geben.
// Jede .go Datei in einem Verzeichnis muss oben das
// gleiche Paket definieren.
// Jede Go Anwendung braucht ein main Paket.
package main

// Wir importieren hier das Paket "fmt"
// Dieses Go-Paket enthält Funktionen zur formatieren
// Ein- und Ausgabe wie beispielsweise fmt.Println(),
// ähnlich wie printf() und scanf() in der
// Programmiersprache C
import "fmt"

func main() {
	fmt.Println("Hallo Welt!")
}
