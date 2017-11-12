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
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  const double min_denom = 0.001;
  double denom = px*px + py*py;

  double denom_sq = sqrt(denom);
  //check division by zero
  if (denom_sq < min_denom) {
    cout << "Division by zero!" << endl;
    denom_sq = min_denom;
  }

  double denom_32 = denom * denom_sq;
  //check division by zero
  if (denom_32 < min_denom) {
    cout << "Division by zero!" << endl;
    denom_32 = min_denom;
  }

  //compute the Jacobian matrix
  Hj << px/denom_sq, py/denom_sq, 0, 0,
    -py/denom, px/denom, 0, 0,
    py*(vx*py - vy*px)/denom_32, px*(vy*px - vx*py)/denom_32, px/denom_sq, py/denom_sq;

  return Hj;
}
