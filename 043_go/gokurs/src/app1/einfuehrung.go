package main

import (
	"app1/demo_eingabe"
	"fmt"
	"github.com/hackebrot/turtle"
	"irgendwas.com/greet"
	"rsc.io/quote"
)

func main() {
	fmt.Println("Willkommen zum Kurs!")

	fmt.Println(quote.Go())

	fmt.Println(greet.GetGreeting())

	emoji, err := turtle.Emojis["smiley"]
	if err {
		fmt.Println("No emoji found.")
	} else {
		fmt.Println(emoji.Char)
	}

	demo_eingabe.Benutzer_kennenlernen()

}
