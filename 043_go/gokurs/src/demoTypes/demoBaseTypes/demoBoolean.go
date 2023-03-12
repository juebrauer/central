package demoBaseTypes

import "fmt"

func ShowDemoBoolean() {

	fmt.Println("Hier ein Demo zum Datentyp Boolean in Go")

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
