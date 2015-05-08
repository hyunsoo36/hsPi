CC = gcc
CXX = g++
CFLAGS = -pipe -Wall -W -O2 -DNO_DEBUG
CXXFLAGS = -pipe -Wall -W -O2 -DNO_DEBUG
WIRINGPIFLAGS =  -lwiringPi
OPENCVFLAGS = -lopencv_core -lopencv_highgui 
RASPIFLAGS = -L/usr/lib/uv4l/uv4lext/armv6l-luv4lext -Wl,-rpath,'/usr/lib/uv4l/uv4lext/armv6l'
THREADPLAGS = -lpthread

SOURCES		=	wing.cpp	hs_udpserver.cpp	hs_serial.cpp	\


OBJECTS		=	wing.o		hs_udpserver.o		hs_serial.o		\


TARGET		=	wing


all: $(OBJECTS)
	$(CXX) $(THREADPLAGS) $(WIRINGPIFLAGS) $(OPENCVFLAGS) $(RASPIFLAGS) -o wing $(OBJECTS)

wing.o : wing.cpp
	$(CXX) $(WIRINGPIFLAGS) $(OPENCVFLAGS) $(RASPIFLAGS) -c wing.cpp
		
hs_udpserver.o : hs_udpserver.cpp
	$(CXX) -c hs_udpserver.cpp

hs_serial.o : hs_serial.cpp
	$(CXX) -c hs_serial.cpp

hs_thread.o : hs_thread.cpp
	$(CXX) $(WIRINGPIFLAGS) $(OPENCVFLAGS) $(RASPIFLAGS) -c hs_thread.cpp

clean: 
	-rm -f $(OBJECTS) $(TARGET) \



