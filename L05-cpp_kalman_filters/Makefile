CXX = /usr/bin/g++
CXXFLAGS = -Wall --std=c++11
SOURCES_01 = ./01_kalman_filter/01_kalman_filter.cpp
SOURCES_02 = ./02_laser_meas/main.cpp ./02_laser_meas/kalman_filter.cpp ./02_laser_meas/tracking.cpp
SOURCES_03 = ./03_jacobian.cpp
SOURCES_04 = ./04_rmse.cpp

01_kalman_filter.o:
	$(CXX) $(CXXFLAGS) -o ./01_kalman_filter.o -I../resources $(SOURCES_01)

02_laser_meas.o:
	$(CXX) $(CXXFLAGS) -o ./02_laser_meas.o -I./02_laser_meas -I../resources $(SOURCES_02)

03_jacobian.o:
	$(CXX) $(CXXFLAGS) -o ./03_jacobian.o -I../resources $(SOURCES_03)

04_rmse.o:
	$(CXX) $(CXXFLAGS) -o ./04_rmse.o -I../resources $(SOURCES_04)

all:

clean:
	rm -rf 01_kalman_filters.o 02_laser_meas.o 03_jacobian.o 04_rmse.o *~
