package greet

import (
	"fmt"
	"math/rand"
	"time"
)

var hello1 string = "Moin Moin!"
var hello2 string = "Servus!"

func GetGreeting() string {

	fmt.Println("GetGreeting v1.0")

	//myRnd := rand.New(rand.NewSource(42))
	myRnd := rand.New(rand.NewSource(time.Now().UnixNano()))
	rndNumber := myRnd.Intn(2)
	fmt.Printf("rndNumber is %d\n", rndNumber)

	if rndNumber == 0 {
		return hello1
	} else {
		return hello2
	}
}
