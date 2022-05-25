all:
	rm -f mdp_simulator
	clear
	@echo "Let us build the project mdp_simulator"
	g++ -Wall *.cpp \
	    -o mdp_simulator \
		`pkg-config --cflags --libs opencv4`
	@echo CODE COMPILED SUCCESSFULLY!