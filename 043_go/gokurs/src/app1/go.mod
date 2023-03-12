module app1

go 1.20

replace irgendwas.com/greet => ../greet

require (
	github.com/hackebrot/turtle v0.2.0
	irgendwas.com/greet v0.0.0-00010101000000-000000000000
	rsc.io/quote v1.5.2
)

require (
	golang.org/x/text v0.3.0 // indirect
	rsc.io/sampler v1.3.0 // indirect
)
