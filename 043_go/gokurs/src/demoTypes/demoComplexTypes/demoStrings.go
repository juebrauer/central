package demoComplexTypes

import "fmt"

func ShowDemoStrings() {

	fmt.Println("Hier ein Demo zu Strings in Go")

	// Definition einer Zeichenkette, Version 1
	txt1 := "Hallo"
	fmt.Println(txt1)

	// Definition einer Zeichenkette, Version 2
	// Die '' sind für runes (Bytes) vorbehalten!
	txt2 := `Hallo`
	fmt.Println(txt2)

	// Wechselseitiges Austauschen von ` und " ist
	// hilfreich bei diversen Texten
	txt3 := `Er sagte "Hallo"`
	fmt.Println(txt3)

	// Wechselseitiges Austauschen von ` und " ist
	// hilfreich bei diversen Texten
	txt4 := "Er sagte `Hallo`"
	fmt.Println(txt4)

	// Wir könnten das Anführungszeichen im Text
	// aber auch über eine Escape-Sequenz erhalten
	txt5 := "Er sagte \"Hallo\""
	fmt.Println(txt5)

	// Umbrechen eines langen Strings in mehre Codezeilen
	txt6 := "In einem Go-Kurs " +
		"ist es erstmal wichtig " +
		"die Grundlagen zu erlernen."
	fmt.Println(txt6)

	// Wir können den String auch fortsetzen, dann geht aber
	// das Einrücken nicht oder wir haben die Sonderzeichen
	// hierfür im eigentlichen String.
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
	var i int
	var c rune
	for i, c = range txt10 {
		fmt.Printf("%d: %s (%d Byte(s))\n", i, string(c), len(string(c)))
	}
	fmt.Println("Länge des Textes", txt10, "in Bytes ist", len(txt10))

	/*
		Quelle ChatGPT:
		Da Unicode-Zeichen möglicherweise mehr
		als ein Byte Speicherplatz benötigen,
		ist es wichtig, sie als rune zu
		behandeln, um sicherzustellen, dass sie
		korrekt dargestellt werden. Mit rune
		können Sie sicherstellen, dass Ihr Code
		Unicode-Zeichen korrekt verarbeitet und
		korrekt ausgibt.
	*/

}
