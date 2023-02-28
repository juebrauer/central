package main

import "fmt"

func main() {

	txt1 := "Hallo"
	fmt.Println(txt1)

	txt2 := `Hallo`
	fmt.Println(txt2)

	txt3 := `Er sagte "Hallo"`
	fmt.Println(txt3)

	txt4 := "Er sagte `Hallo`"
	fmt.Println(txt4)

	txt5 := "Er sagte \"Hallo\""
	fmt.Println(txt5)

	txt6 := "In einem Go-Kurs " +
		"ist es erstmal wichtig " +
		"die Grundlagen zu erlernen."
	fmt.Println(txt6)

	txt7 := `In einem Go-Kurs
ist es erstmal wichtig
die Grundlagen zu erlernen.`
	fmt.Println(txt7)

	// Strings mit `` sind nicht interpretiert:
	txt8 := `Hier kommt eine Escape Sequenz: \n`
	fmt.Println(txt8)

	// Strings mit "" sind interpretiert:
	txt9 := "Hier kommt eine Escape Sequenz: \n"
	fmt.Println(txt9)

	/*
		UTF-8 is an encoding scheme used to encode
		variable width characters into one to four bytes.
		Go supports UTF-8 characters out of the box,
		without any special setup, libaries, or packages.
	*/
	txt10 := "Hello 世界!"
	fmt.Println(txt10)

	for i, c := range txt10 {
		fmt.Printf("%d: %s (%d Byte(s))\n", i, string(c), len(string(c)))
	}
	fmt.Println("Länge des Textes", txt10, "in Bytes ist", len(txt10))
}
