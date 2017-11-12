#include <iostream>
#include <algorithm>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
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

double Tools::CalculateNISReport(const vector<double> &NIS_vector, const double reference) {
  auto count = std::count_if(NIS_vector.begin(), NIS_vector.end(), [reference](double NIS) { return NIS > reference; });
  return static_cast<double>(count)/NIS_vector.size();
}
