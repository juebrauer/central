package main

import "fmt"

func main() {

	// Diese Anweisung deklariert eine Variable i1
	// und initialisiert sie gleich mit 1
	var i1 int = 1
	fmt.Println(i1)

	// Diese Anweisung deklariert und initialisiert eine
	// Variable i2, ohne, dass wir ihren Typ explizit
	// angeben
	i2 := 2
	fmt.Println(i2)

	// Diese Anweisung deklariert und initialisiert eine
	// Variable f1 vom Typ float
	f1 := 3.14159
	fmt.Println(f1)

	// Go unterscheidet zwischen
	// Architektur-unabhängigen (architecture independent)
	// Datentypen wie int32, int64
	// und
	// Implementierungs-spezifischen Datentypen wie int
	// Hier kommt es darauf an, ob die Zielplattform
	// 32Bit (Fitness-Watch) oder 64Bit (PC Betriebssysteme)
	// verwendet
	var i4 int32 = 10
	var i5 int64 = 20
	i6 := int64(20)
	fmt.Println(i4, i5, i6)

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
	i7 := uint(255)
	i8 := byte(255)
	i9 := rune(255)
	fmt.Println(i7, i8, i9)
	fmt.Println(i7-256, i8+1, i9+1)

	var i10 uint32 = 4294967295 // größte Zahl, die ein uint32 speichern kann
	fmt.Println(i10)
	fmt.Println(i10 + 1)

	//var i11 uint32 = 4294967295 + 1 // Hier beschwert sich der Compiler + die IDE!
	//fmt.Println(i11)

	c1 := complex(10, 99) // constructor init
	c2 := 10 + 99i        // complex number init syntax
	fmt.Println(c1, c2, c1+c2, c1*c1)
	fmt.Println(real(c1))
	fmt.Println(imag(c1))

	f2 := float32(3.14159)
	fmt.Println(f2)
}
