CC = gcc
CXX = g++
CFLAGS = -pipe -Wall -W -O2 -DNO_DEBUG
CXXFLAGS = -pipe -Wall -W -O2 -DNO_DEBUG
WIRINGPIFLAGS =  -lwiringPi
OPENCVFLAGS = -lopencv_core -lopencv_highgui 
RASPIFLAGS = -L/usr/lib/uv4l/uv4lext/armv6l-luv4lext -Wl,-rpath,'/usr/lib/uv4l/uv4lext/armv6l'



SOURCES		=	wing.cpp		\


OBJECTS		=	wing.o			\


TARGET		=	wing



al l: $(OBJECTS)
	g++ -lwiringPi -lopencv_core -lopencv_highgui -L/usr/lib/uv4l/uv4lext/armv6l-luv4lext -Wl,-rpath,'/usr/lib/uv4l/uv4lext/armv6l' wing.cpp -o wing

wing.o : wing.cpp
	$(CXX) $(WIRINGPIFLAGS) $(OPENCVFLAGS) $(RASPIFLAGS) -o wing 
		

clean: 
	-rm -f $(OBJECTS) $(TARGET) \



