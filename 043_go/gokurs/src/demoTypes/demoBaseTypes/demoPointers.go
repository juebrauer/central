package demoBaseTypes

import "fmt"

func sliceMultiplierV1(S []int) {
	for i := 0; i < len(S); i++ {
		S[i] *= 2
	}
}

func arrayMultiplierV1(A [5]int) {
	for i := 0; i < len(A); i++ {
		A[i] *= 2
	}
}

func arrayMultiplierV2(A *[5]int) {
	for i := 0; i < len(A); i++ {
		(*A)[i] *= 2
	}
}

func ShowDemoPointers() {

	// Wie kann man einen Pointer definieren und initialisieren?
	fmt.Println("A")
	var x int = 42
	var p1 *int = &x
	var p2 = &x
	fmt.Println("x=", x)
	fmt.Println("*p1=", *p1)
	fmt.Println("*p2=", *p2)

	// Verändern der Variable über den Pointer
	fmt.Println("B")
	*p1 = 43
	fmt.Println("x=", x)
	fmt.Println("*p1=", *p1)
	fmt.Println("*p2=", *p2)

	// Speicheradressen ausgeben
	fmt.Println("C")
	fmt.Println(&x)
	fmt.Println(p1)
	fmt.Println(p2)

	// Unterschied zu Pointern in C:
	// Pointer-Arithmetik ist nicht möglich
	//var p3 *int = p1+p2

	// Aber: Pointer-auf-Pointer geht auch!
	fmt.Println("D")
	var p3 **int = &p1
	fmt.Println(**p3)

	// Wichtiger Grund für die Existenz von Pointern
	// in Go: Das Vermeiden des Kopierens großer
	// Datenstrukturen wie Arrays, wenn wir
	// diese an eine Funktion übergeben

	// Slices werden in Go im Wesentlichen
	// per Call-by-reference an Funktionen übergeben
	// "Im Wesentlichen", weil eine Kopie des Slices Headers
	// übergeben wird. Aber das sind Details.
	// Die darunterliegende Datenstruktur wird auf jeden Fall
	// nicht kopiert!
	fmt.Println("E")
	var S1 []int = []int{1, 2, 3, 4, 5}
	fmt.Println("S1 vorher:", S1)
	sliceMultiplierV1(S1)
	fmt.Println("S1 nachher:", S1)

	// Arrays dagegen werden per Call-by-Value übergeben
	// --> alle Daten werden kopiert!
	// Das folgende Beispiel zeigt, dass das Ausgangs-Array
	// nicht modifiziert wird, nur die lokale Kopie des Arrays
	// in arrayMultiplierV1
	fmt.Println("F")
	var A1 [5]int = [5]int{1, 2, 3, 4, 5}
	fmt.Println("A1 vorher:", A1)
	arrayMultiplierV1(A1)
	fmt.Println("A1 nachher:", A1)

	// Jetzt nochmal, aber dieses Mal per Call-by-reference
	// --> das Ausgangsarray wird modifiziert
	fmt.Println("G")
	fmt.Println("A1 vorher:", A1)
	arrayMultiplierV2(&A1)
	fmt.Println("A1 nachher:", A1)
}
