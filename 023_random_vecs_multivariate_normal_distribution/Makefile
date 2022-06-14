all:
	rm -f kalman_filter_2d_demo
	clear
	@echo "Let us build the project kalman_filter_2d_demo"
	g++ -Wall *.cpp \
	    -o kalman_filter_2d_demo \
		`pkg-config --cflags --libs opencv4`
	@echo CODE COMPILED SUCCESSFULLY!

mvnrnd:
	rm -f mvnrnd
	clear
	@echo "Building subproject mvnrnd_demo"
	g++ -Wall mvnrnd.cpp mvnrnd_demo.cpp \
	    -o mvnrnd_demo \
		`pkg-config --cflags --libs opencv4`
	@echo "Finished building mvnrnd_demo"
