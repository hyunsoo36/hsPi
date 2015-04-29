g++ -lwiringPi -lopencv_core -lopencv_highgui -L/usr/lib/uv4l/uv4lext/armv6l-luv4lext -Wl,-rpath,'/usr/lib/uv4l/uv4lext/armv6l' wing.cpp -o wing
