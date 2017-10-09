#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  //measurement transition matrix - laser
  H_laser_ << 1, 0, 0, 0,
    0, 1, 0, 0;
}


/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
    */
    // first measurement
    VectorXd x_in(4);
    MatrixXd P_in(4, 4);
    MatrixXd Q_in(4, 4);
    MatrixXd F_in(4, 4);
    P_in << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;
    Q_in << 0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0;
    F_in << 1, 0, 1, 0,
      0, 1, 0, 1,
      0, 0, 1, 0,
      0, 0, 0, 1;

    previous_timestamp_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double px = measurement_pack.raw_measurements_(0) * std::cos(measurement_pack.raw_measurements_(1));
      double py = measurement_pack.raw_measurements_(0) * std::sin(measurement_pack.raw_measurements_(1));
      double vx = measurement_pack.raw_measurements_(2) * std::cos(measurement_pack.raw_measurements_(1));
      double vy = measurement_pack.raw_measurements_(2) * std::sin(measurement_pack.raw_measurements_(1));
      x_in << px, py, vx, vy;
      ekf_.Init(x_in, P_in, F_in, Hj_, R_radar_, Q_in);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      x_in << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), 0, 0;
      ekf_.Init(x_in, P_in, F_in, H_laser_, R_laser_, Q_in);
    }

    // ekf_.setDebug(true);

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
   */
  double dt = static_cast<double>(measurement_pack.timestamp_ - previous_timestamp_)/1000000;
  previous_timestamp_ = measurement_pack.timestamp_;
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  double noise_ax = 9;
  double noise_ay = 9;
  ekf_.Q_(0,0) = std::pow(dt,4)*noise_ax/4;
  ekf_.Q_(0,2) = std::pow(dt,3)*noise_ax/2;
  ekf_.Q_(1,1) = std::pow(dt,4)*noise_ay/4;
  ekf_.Q_(1,3) = std::pow(dt,3)*noise_ay/2;
  ekf_.Q_(2,0) = std::pow(dt,3)*noise_ax/2;
  ekf_.Q_(2,2) = std::pow(dt,2)*noise_ax;
  ekf_.Q_(3,1) = std::pow(dt,3)*noise_ay/2;
  ekf_.Q_(3,3) = std::pow(dt,2)*noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << "\n\n" << endl;
}
