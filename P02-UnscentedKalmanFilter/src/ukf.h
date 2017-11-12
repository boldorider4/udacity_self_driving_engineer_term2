#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* augmented state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate v_a v_yawdd] in SI units and rad
  VectorXd x_aug_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* augmented state covariance matrix
  MatrixXd P_aug_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* radar measurement noise matrix
  MatrixXd R_radar_;

  ///* lidar measurement noise matrix
  MatrixXd R_laser_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;


  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(const MeasurementPackage& meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(const double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser or radar
   * measurement
   * @param meas_package The measurement at k+1
   */
  void Update(const MeasurementPackage& meas_package);

private:
  void ComputeZsig(const MeasurementPackage& meas_package, MatrixXd& Zsig);
  void ComputeS(const MeasurementPackage& meas_package, const MatrixXd& Zsig, const VectorXd& z_pred, MatrixXd& S);
  void ComputeTc(const MeasurementPackage& meas_package, const MatrixXd& Zsig, const VectorXd& z_pred, MatrixXd& Tc);
};

#endif /* UKF_H */
