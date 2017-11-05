#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>
#include <stdexcept>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // Complete the initialization. See ukf.h for other member properties.
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;
  time_us_ = 0;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // state vector
  x_ = VectorXd(n_x_);
  x_.fill(0.0);

  // augmented state vector
  x_aug_ = VectorXd(n_aug_);
  x_aug_.fill(0.0);

  // augmented covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  P_.fill(0.0);

  // augmented covariance matrix
  P_aug_ = MatrixXd(n_aug_, n_aug_);
  P_aug_.fill(0.0);

  // Predicted sigma points
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_ + 1);
  Xsig_pred_.fill(0);

  // radar measurement noise matrix
  R_radar_ = MatrixXd(3,3);
  R_radar_.fill(0.0);

  // radar measurement noise matrix
  R_laser_ = MatrixXd(2,2);
  R_laser_.fill(0.0);

  // weigths
  weights_ = VectorXd(2*n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (size_t ix = 1; ix < 2*n_aug_ + 1; ix++) {
    weights_(ix) = 0.5/(n_aug_ + lambda_);
  }

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 6;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 10;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  // fill invariant values of P_aug_
  P_aug_(5,5) = std_a_*std_a_;
  P_aug_(6,6) = std_yawdd_*std_yawdd_;

  // fill invariant values of R_radar_
  R_radar_(0,0) = std_radr_*std_radr_;
  R_radar_(1,1) = std_radphi_*std_radphi_;
  R_radar_(2,2) = std_radrd_*std_radrd_;

  // fill invariant values of R_laser_
  R_laser_(0,0) = std_laspx_*std_laspx_;
  R_laser_(1,1) = std_laspy_*std_laspy_;

  // flag for initialization
  is_initialized_ = false;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(const MeasurementPackage& meas_package) {
  /*
    Complete this function! Make sure you switch between lidar and radar
    measurements.
  */
  if ((use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) || \
      (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER)) {

    if (!is_initialized_) {

      if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        /*
          Convert radar from polar to cartesian coordinates and initialize state.
        */
        x_(0) = meas_package.raw_measurements_(0) * std::cos(meas_package.raw_measurements_(1));
        x_(1) = meas_package.raw_measurements_(0) * std::sin(meas_package.raw_measurements_(1));
      } else {
        /*
          Initialize state.
        */
        x_(0) = meas_package.raw_measurements_(0);
        x_(1) = meas_package.raw_measurements_(1);
      }

      time_us_ = meas_package.timestamp_;

      is_initialized_ = true;
    }

    /* prediction */
    double dt = static_cast<double>(meas_package.timestamp_ - time_us_)/1000000;
    time_us_ = meas_package.timestamp_;

    Prediction(dt);

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      UpdateRadar(meas_package);
    } else {
      UpdateLidar(meas_package);
    }
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /*
    Estimate the object's location.
  */
  x_aug_.head(n_x_) << x_;
  P_aug_.topLeftCorner(n_x_,n_x_) << P_;

  /* generate sigma points */
  MatrixXd Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  MatrixXd L = P_aug_.llt().matrixL();

  Xsig_aug_.col(0)  = x_aug_;
  for (int col = 0; col < n_aug_; col++)
  {
    Xsig_aug_.col(col+1)        = x_aug_ + sqrt(lambda_+n_aug_) * L.col(col);
    Xsig_aug_.col(col+1+n_aug_) = x_aug_ - sqrt(lambda_+n_aug_) * L.col(col);
  }

  /* predict sigma points */
  double p_x, p_y, v, psi, psi_dot, n_a, n_psi, px_term, py_term;

  for (size_t col = 0; col < (size_t)(2*n_aug_+1); col++) {
    p_x = Xsig_aug_(0,col);
    p_y = Xsig_aug_(1,col);
    v   = Xsig_aug_(2,col);
    psi = Xsig_aug_(3,col);
    psi_dot = Xsig_aug_(4,col);
    n_a = Xsig_aug_(5,col);
    n_psi = Xsig_aug_(6,col);

    /* avoid zero division */
    px_term = (psi_dot < 0.001) ? v*std::cos(psi) * delta_t :
                                           (v/psi_dot) * (std::sin(psi + psi_dot*delta_t) - std::sin(psi));
    py_term = (psi_dot < 0.001) ? v*std::sin(psi) * delta_t :
                                           (v/psi_dot) * (std::cos(psi) - std::cos(psi + psi_dot*delta_t));

    Xsig_pred_(0, col) = p_x + px_term + std::cos(psi) * n_a * delta_t * delta_t * .5;
    Xsig_pred_(1, col) = p_y + py_term + std::sin(psi) * n_a * delta_t * delta_t * .5;
    Xsig_pred_(2, col) = v + delta_t * n_a;
    Xsig_pred_(3, col) = psi + psi_dot * delta_t + n_psi * delta_t * delta_t * .5;
    Xsig_pred_(4, col) = psi_dot + delta_t * n_psi;
  }

  /* predict mean state and covariance matrix */
  x_.fill(0.0);
  P_.fill(0.0);

  //predict state mean
  for (size_t col = 0; col < (size_t)(2*n_aug_+1); col++) {
    x_ += weights_(col) * Xsig_pred_.col(col);
  }

  //predict state covariance matrix
  for (size_t col = 0; col < (size_t)(2*n_aug_+1); col++) {
    VectorXd diff = Xsig_pred_.col(col) - x_;
    while (diff(3) > M_PI) diff(3) -= 2.*M_PI;
    while (diff(3) < -M_PI) diff(3) += 2.*M_PI;
    P_ += weights_(col) * diff * diff.transpose();;
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(const MeasurementPackage& meas_package) {
  /*
    Use lidar data to update the belief about the object's
    position. Calculate the lidar NIS.
  */
  int n_z = 2;

  VectorXd z_pred = VectorXd(n_z);
  MatrixXd S = MatrixXd(n_z,n_z);

  z_pred.fill(0.0);
  //calculate mean predicted measurement
  for (size_t col = 0; col < 2 * n_aug_ + 1; col++) {
    z_pred += weights_(col) * Xsig_pred_.col(col).head(2);
  }

  //calculate measurement covariance matrix S
  S.fill(0.0);
  for (size_t col = 0; col < 2 * n_aug_ + 1; col++) {
    VectorXd z_diff = Xsig_pred_.col(col).head(2) - z_pred;
    S += weights_(col) * z_diff * z_diff.transpose();
  }

  S += R_laser_;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (size_t col = 0; col < 2 * n_aug_ + 1; col++) {
    VectorXd x_diff = Xsig_pred_.col(col) - x_;
    VectorXd z_diff = Xsig_pred_.col(col).head(2) - z_pred;
    Tc += weights_(col) * x_diff * z_diff.transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //update state mean and covariance matrix
  x_ += K * (meas_package.raw_measurements_ - z_pred);
  P_ -= K * S * K.transpose();
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(const MeasurementPackage& meas_package) {
  /*
    Use radar data to update the belief about the object's
    position. You'll also need to calculate the radar NIS.
  */
  int n_z = 3;

  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  VectorXd z_pred = VectorXd(n_z);
  MatrixXd S = MatrixXd(n_z,n_z);

  for (size_t col = 0; col < 2 * n_aug_ + 1; col++) {
    double p_x = Xsig_pred_(0,col);
    double p_y = Xsig_pred_(1,col);
    double v   = Xsig_pred_(2,col);
    double psi = Xsig_pred_(3,col);

    Zsig(0,col) = std::sqrt(p_x * p_x + p_y * p_y);
    Zsig(1,col) = std::atan2(p_y, p_x);
    Zsig(2,col) = v * (p_x * std::cos(psi) + p_y * std::sin(psi)) / Zsig(0,col);
  }

  z_pred.fill(0.0);
  //calculate mean predicted measurement
  for (size_t col = 0; col < 2 * n_aug_ + 1; col++) {
    z_pred += weights_(col) * Zsig.col(col);
  }

  //calculate measurement covariance matrix S
  S.fill(0.0);
  for (size_t col = 0; col < 2 * n_aug_ + 1; col++) {
    VectorXd z_diff = Zsig.col(col) - z_pred;

    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S += weights_(col) * z_diff * z_diff.transpose();
  }

  S += R_radar_;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (size_t col = 0; col < 2 * n_aug_ + 1; col++) {
    VectorXd x_diff = Xsig_pred_.col(col) - x_;
    VectorXd z_diff = Zsig.col(col) - z_pred;

    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    Tc += weights_(col) * x_diff * z_diff.transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //update state mean and covariance matrix
  x_ += K * (meas_package.raw_measurements_ - z_pred);
  P_ -= K * S * K.transpose();
}
