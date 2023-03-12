package demo_eingabe

import (
	"bufio"
	"errors"
	"fmt"
	"os"
	"strings"
)

func frage_nach_name() string {

	frage := "Wie heißt du denn? "
	fmt.Print(frage)

	var name string
	n, err := fmt.Scanln(&name)
	if err != nil {
		fmt.Println("\tUps! Da ist wohl was schief gelaufen!")
		fmt.Println("\tEingelesene Items:", n)
		fmt.Println("\tFehler:", err)
		os.Exit(1)
	}

	fmt.Printf("Ich habe %d viele Dinge eingelesen.\n", n)
	return name
}

func frage_nach_alter() (int, error) {

	frage := "Wie alt bist du? "
	fmt.Println(frage)

	var alter int
	_, err := fmt.Scanln(&alter)

	if err != nil {
		fmt.Println("Fehler: ", err)
		os.Exit(1)
	}

	if alter > 122 {
		fehler_txt := fmt.Sprintf("Glaube ich nicht, dass du %d Jahre alt bist", alter)
		return alter, errors.New(fehler_txt)
	} else {
		return alter, nil
	}
}

func frage_nach_hobbies() []string {
	frage := "Was hast du denn für Hobbies?"
	fmt.Println(frage)

	reader := bufio.NewReader(os.Stdin)

	var zeile string
	var err error
	zeile, err = reader.ReadString('\n')
	if err != nil {
		fmt.Println("Fehler:", err)
		os.Exit(1)
	}
	fmt.Printf("Eingelesen: %s\n", zeile)

	/*
		my_string_splitter1 := func(c rune) bool {
			return c == ' ' || c == ','
		}
	*/

	// FieldsFunc() nutzt eine Hilfsfunktion,
	// um zu entscheiden, wo der String gesplittet
	// werden soll
	//hobbies := strings.FieldsFunc(zeile, my_string_splitter1)
	hobbies := strings.FieldsFunc(zeile, my_string_splitter2)

	return hobbies
}

func Benutzer_kennenlernen() {

	name := frage_nach_name()
	fmt.Printf("Hallo %s!\n", name)

	alter, err := frage_nach_alter()
	if err != nil {
		fmt.Println("Fehler bei der Eingabe des Alters!")
		fmt.Println(err)
		os.Exit(1)
	}
	fmt.Printf("So, so... %d Jahre alt...\n", alter)
	if alter > 40 {
		fmt.Print("Wow! ")
	}
	tage := alter * 365
	fmt.Printf("Dann bist du also schon %d Tage alt\n", tage)

	hobbies := frage_nach_hobbies()
	fmt.Println(hobbies)

	txt1 := fmt.Sprintf("Du hast also %d Hobbies", len(hobbies))
	fmt.Println(txt1)
	for i, v := range hobbies {
		txt2 := fmt.Sprintf("Hobby #%d: %s", i, v)
		fmt.Println(txt2)
	}

}
