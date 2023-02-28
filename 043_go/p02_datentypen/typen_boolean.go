package main

import "fmt"

func main() {

	// print() und fmt.Print() machen kein Newline
	// am Ende und kein Space zwischen Argumenten
	print("A", "B", "C")
	fmt.Print("D", "E", "F")

	// ... dagegen println() und fmt.Println() schon
	println("A", "B", "C")
	fmt.Println("D", "E", "F")

	// Achtung! Man sollte fmt.Print() und fmt.Println()
	// verwenden statt print() und println() denn:
	// "Current implementations provide several built-in functions
	// useful during bootstrapping. These functions are documented
	// for completeness but are not guaranteed to stay in the language.
	// They do not return a result."

	var b1 bool = false
	fmt.Println("b1 =", b1)

	var b2 bool = true
	fmt.Println("b2 =", b2)

	b3 := b1 || b2
	fmt.Println("b3 =", b3)

	b4 := b1 && b2
	fmt.Println("b4 =", b4)

	b5 := !b1
	fmt.Println("b5 =", b5)
}
