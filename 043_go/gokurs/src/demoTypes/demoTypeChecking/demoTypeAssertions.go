package demoTypeChecking

import "fmt"

func processValue(i interface{}) {
	switch v := i.(type) {
	case int:
		fmt.Println("ein Integer:", v)
	case float32, float64:
		fmt.Println("ein Float:", v)
	case string:
		fmt.Println("ein String:", v)
	case []int:
		fmt.Println("eine int Slice", v)
	default:
		fmt.Println("Unbekannter Typ:", v)
	}
}
func ShowDemoTypeAssertions() {

	fmt.Println("Ein Demo zu Type Assertions")

	// Die Funktion processValue kann mit unterschiedlichen
	// Datentypen arbeiten!
	// Jeder eingebaute Datentyp in Go implementiert
	// nämlich das "leere Interface"
	processValue(int(42))
	processValue(int32(42))
	processValue("Hello, world!")
	processValue(3.14159)
	processValue([]int{10, 20, 30})
	processValue([5]int{10, 20, 30})
	processValue(true)
}
