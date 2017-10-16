CXX = /usr/bin/g++
CXXFLAGS = -Wall --std=c++11
SOURCES_01 = ./01_sigma_points/main.cpp ./01_sigma_points/ukf.cpp

01_sigma_points.o:
	$(CXX) $(CXXFLAGS) -o ./01_sigma_points_o -I../resources/Eigen -I./01_sigma_points $(SOURCES_01)

all:

clean:
	rm -rf 01_sigma_points.o *~
