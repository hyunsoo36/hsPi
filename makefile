CC = gcc
CXX = g++
CFLAGS = -pipe -Wall -W -O2 -DNO_DEBUG
CXXFLAGS = -pipe -Wall -W -O2 -DNO_DEBUG
WIRINGPIFLAGS =  -lwiringPi
OPENCVFLAGS = -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_video
RASPIFLAGS = -L/usr/lib/uv4l/uv4lext/armv6l-luv4lext -Wl,-rpath,'/usr/lib/uv4l/uv4lext/armv6l'
THREADPLAGS = -pthread

SOURCES		=	wing.cpp	hs_udpserver.cpp	hs_serial.cpp	hs_thread.cpp	hsNavi.cpp	\


OBJECTS		=	wing.o		hs_udpserver.o		hs_serial.o		hs_thread.o		hsNavi.o	\


TARGET		=	wing


all: $(OBJECTS)
	$(CXX) -Wall $(OBJECTS) $(THREADPLAGS) $(WIRINGPIFLAGS) $(OPENCVFLAGS) $(RASPIFLAGS) -o wing 

wing.o : wing.cpp
	$(CXX) $(THREADPLAGS) $(OPENCVFLAGS) $(RASPIFLAGS) -c wing.cpp
		
hs_udpserver.o : hs_udpserver.cpp
	$(CXX) -c hs_udpserver.cpp

hs_serial.o : hs_serial.cpp
	$(CXX) -c hs_serial.cpp

hs_thread.o : hs_thread.cpp
	$(CXX) $(WIRINGPIFLAGS) $(OPENCVFLAGS) $(RASPIFLAGS) -c hs_thread.cpp

hsNavi.o : hsNavi.cpp
	$(CXX) -c hsNavi.cpp

clean: 
	-rm -f $(OBJECTS) $(TARGET) \



