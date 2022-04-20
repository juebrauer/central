# You need to install OpenCV to compile this code, e.g. with:
#    sudo apt install libopencv-dev

all:
	rm -f robot_navigation_with_a_map
	clear
	@echo "Let us build the project robot_navigation_with_a_map"
	g++ -Wall *.cpp \
	    -o robot_navigation_with_a_map \
		`pkg-config --cflags --libs opencv4`
	@echo CODE COMPILED SUCCESSFULLY!