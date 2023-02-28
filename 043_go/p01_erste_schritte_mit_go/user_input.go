package main

import (
	"fmt"
)

func main() {
	fmt.Println("Bitte gib deinen Namen ein:")
	var name string
	fmt.Scanln(&name)
	fmt.Printf("Hallo, %s! Hoffe, es geht dir gut!", name)
}
