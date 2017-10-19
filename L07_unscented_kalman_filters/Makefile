CXX = /usr/bin/g++
CXXFLAGS = -Wall --std=c++11
SOURCES_01 = ./01_sigma_points/main.cpp ./01_sigma_points/ukf.cpp
SOURCES_02 = ./02_augmentation/main.cpp ./02_augmentation/ukf.cpp
SOURCES_03 = ./03_sigma_prediction/main.cpp ./03_sigma_prediction/ukf.cpp

01_sigma_points.o:
	$(CXX) $(CXXFLAGS) -o ./01_sigma_points_o -I../resources/Eigen -I./01_sigma_points $(SOURCES_01)

02_augmentation.o:
	$(CXX) $(CXXFLAGS) -o ./02_augmentation_o -I../resources/Eigen -I./02_augmentation $(SOURCES_02)

03_sigma_prediction.o:
	$(CXX) $(CXXFLAGS) -o ./03_sigma_prediction_o -I../resources/Eigen -I./03_sigma_prediction $(SOURCES_03)

all: 01_sigma_points.o 02_augmentation.o 03_sigma_prediction.o

clean:
	rm -rf 01_sigma_points.o 02_augmentation_o 03_sigma_prediction.o *~
