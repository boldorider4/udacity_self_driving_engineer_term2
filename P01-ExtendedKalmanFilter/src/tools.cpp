#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (!estimations.size() || estimations.size() != ground_truth.size()) {
    cout << "estimation size is zero or different than ground truth" << endl;
  }

  //accumulate squared residuals
  for(size_t idx = 0; idx < estimations.size(); idx++) {
    VectorXd residual = estimations[idx]-ground_truth[idx];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  //calculate the mean
  rmse = rmse/estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);

  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float denom = px*px + py*py;
  //check division by zero
  if (denom < 0.0001f) {
    cout << "Division by zero!" << endl;
    Hj << 0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0;
    return Hj;
  }

  //compute the Jacobian matrix
  float denom_sq = sqrt(denom);
  float denom_32 = denom * denom_sq;
  Hj << px/denom_sq, py/denom_sq, 0, 0,
    -py/denom, px/denom, 0, 0,
    py*(vx*py - vy*px)/denom_32, px*(vy*px - vx*py)/denom_32, px/denom_sq, py/denom_sq;

  return Hj;
}
