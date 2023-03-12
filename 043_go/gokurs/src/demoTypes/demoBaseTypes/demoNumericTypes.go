package demoBaseTypes

import (
	"fmt"
	"unsafe"
)

/*
	uint8       unsigned  8-bit integers (0 to 255)
	uint16      unsigned 16-bit integers (0 to 65535)
	uint32      unsigned 32-bit integers (0 to 4294967295)
	uint64      unsigned 64-bit integers (0 to 18446744073709551615)
	int8        signed  8-bit integers (-128 to 127)
	int16       signed 16-bit integers (-32768 to 32767)
	int32       signed 32-bit integers (-2147483648 to 2147483647)
	int64       signed 64-bit integers (-9223372036854775808 to 9223372036854775807)
	float32     IEEE-754 32-bit floating-point numbers
	float64     IEEE-754 64-bit floating-point numbers
	complex64   complex numbers with float32 real and imaginary parts
	complex128  complex numbers with float64 real and imaginary parts

	Andere Bezeichnungen ("alias"):
	byte        alias for uint8
	rune        alias for int32
*/

func ShowDemoBaseTypes() {
	fmt.Println("Hier ein Demo zu numerischen Basisdatentypen")

	// A: Keine direkte Angabe der Anzahl der Bits bei Integer
	var i1 int = 1
	fmt.Println("\nA", i1)
	size := unsafe.Sizeof(i1)
	fmt.Println("Größe in Bytes", size)
	// Auf einem 64-Bit-System sehen wir als Größe: 8 Bytes
	// Wenn wir unser Programm als 32-Bit-Anwendung kompilieren
	// z.B. mit
	// GOARCH=386 go build
	// sehen wir dagegen: 4 Bytes

	// B: Direkte Angabe der Anzahl der Bits bei Integer
	var i2 int32 = 2
	fmt.Println("\nB", i2)
	size = unsafe.Sizeof(i2)
	fmt.Println("Größe in Bytes", size)

	// C: Direkte Angabe der Anzahl der Bits bei Integer
	var i3 int64 = 2
	fmt.Println("\nC", i3)
	size = unsafe.Sizeof(i3)
	fmt.Println("Größe in Bytes", size)

	// D: Andere Möglichkeit, ein int zu deklarieren
	var i4 = int64(12345)
	fmt.Println("\nD", i4)

	// E: Overflow
	i5 := uint32(4294967295) // größte Zahl, die ein uint32 speichern kann)
	//i5 := uint32(4294967296) // da meckert schon die Entwicklungsumgebung!
	fmt.Println("\nE", i5, i5+1, i5+2)

	// F: floats
	// f1 := float(3.14159) // gibt es nicht in Go!
	f2 := float32(3.14159)
	f3 := float64(3.14159)
	fmt.Println("\nF", f2, f3)
	fmt.Println("Größen in Bytes")
	fmt.Println(unsafe.Sizeof(f2), unsafe.Sizeof(f3))

	// G: Komplexe Zahlen
	fmt.Println("\nG")
	c1 := complex(10, 99) // Initialisierung über Konstruktor
	c2 := 10 + 99i        // Initialisierung über arithmetischen Ausdruck und := Operator
	fmt.Println(c1, c2, c1+c2, c1*c1)
	fmt.Println(real(c1))
	fmt.Println(imag(c1))
}
