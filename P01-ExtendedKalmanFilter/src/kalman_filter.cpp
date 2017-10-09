#include "kalman_filter.h"
#include <cmath>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() : debug(0) {}
KalmanFilter::KalmanFilter(bool debug) : debug(debug) {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::setDebug(bool debug) {
  this->debug = debug;
}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
    * predict the state
  */
  if (debug) {
    std::cout << "**PREDICTION-STEP**" << std::endl;
    std::cout << "F_" << F_ << std::endl;
    std::cout << "x_ before prediction " << x_ << std::endl;
    std::cout << "P_ before prediction " << P_ << std::endl;
  }

  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;

  if (debug) {
    std::cout << "x_ after prediction " << x_ << std::endl;
    std::cout << "P_ after prediction " << P_ << std::endl;
  }
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */
  VectorXd pred = H_ * x_;

  if (debug) {
    std::cout << "\n**CORRECTION-STEP: Lidar**" << std::endl;
    std::cout << "measurement " << z << std::endl;
  }

  measurementUpdateCore(z, pred);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd pred(3);
  pred(0) = std::sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  pred(1) = std::atan2(x_(1),x_(0));
  pred(2) = (x_(0)*x_(2) + x_(1)*x_(3))/pred(0);

  if (debug) {
    std::cout << "**CORRECTION-STEP: Radar**" << std::endl;
    std::cout << "measurement " << z << std::endl;
  }

  measurementUpdateCore(z, pred);
}

void KalmanFilter::measurementUpdateCore(const VectorXd &z, const VectorXd &prediction) {
  VectorXd y = z - prediction;
  MatrixXd Ht = H_.transpose();
  MatrixXd K = H_ * P_ * Ht + R_;
  K = K.inverse();
  K = P_ * Ht * K;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}
