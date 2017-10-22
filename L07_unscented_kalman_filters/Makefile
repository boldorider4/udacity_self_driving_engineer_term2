CXX = /usr/bin/g++
CXXFLAGS = -Wall --std=c++11
SOURCES_01 = ./01_sigma_points/main.cpp ./01_sigma_points/ukf.cpp
SOURCES_02 = ./02_augmentation/main.cpp ./02_augmentation/ukf.cpp
SOURCES_03 = ./03_sigma_prediction/main.cpp ./03_sigma_prediction/ukf.cpp
SOURCES_04 = ./04_predicted_mean_covariance/main.cpp ./04_predicted_mean_covariance/ukf.cpp
SOURCES_05 = ./05_radar_measurement_prediction/main.cpp ./05_radar_measurement_prediction/ukf.cpp
SOURCES_06 = ./06_ukf_update/main.cpp ./06_ukf_update/ukf.cpp

01_sigma_points.o:
	$(CXX) $(CXXFLAGS) -o ./01_sigma_points_o -I../resources/Eigen -I./01_sigma_points $(SOURCES_01)

02_augmentation.o:
	$(CXX) $(CXXFLAGS) -o ./02_augmentation_o -I../resources/Eigen -I./02_augmentation $(SOURCES_02)

03_sigma_prediction.o:
	$(CXX) $(CXXFLAGS) -o ./03_sigma_prediction_o -I../resources/Eigen -I./03_sigma_prediction $(SOURCES_03)

04_predicted_mean_covariance.o:
	$(CXX) $(CXXFLAGS) -o ./04_predicted_mean_covariance_o -I../resources/Eigen -I./04_predicted_mean_covariance $(SOURCES_04)

05_radar_measurement_prediction.o:
	$(CXX) $(CXXFLAGS) -o ./05_radar_measurement_prediction_o -I../resources/Eigen -I./05_radar_measurement_prediction $(SOURCES_05)

06_ukf_update.o:
	$(CXX) $(CXXFLAGS) -o ./06_ukf_update_o -I../resources/Eigen -I./06_ukf_update $(SOURCES_06)

all: 01_sigma_points.o 02_augmentation.o 03_sigma_prediction.o 04_predicted_mean_covariance.o 05_radar_measurement_prediction.o 06_ukf_update.o

clean:
	rm -rf 01_sigma_points.o 02_augmentation_o 03_sigma_prediction.o 04_predicted_mean_covariance.o 05_radar_measurement_prediction.o 06_ukf_update.o *~
