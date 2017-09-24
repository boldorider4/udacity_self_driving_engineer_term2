#include "Eigen/Dense"
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd CalculateJacobian(const VectorXd& x_state);

int main() {

	/*
	 * Compute the Jacobian Matrix
	 */

	//predicted state  example
	//px = 1, py = 2, vx = 0.2, vy = 0.4
	VectorXd x_predicted(4);
	x_predicted << 1, 2, 0.2, 0.4;

	MatrixXd Hj = CalculateJacobian(x_predicted);

	cout << "Hj:" << endl << Hj << endl;

	return 0;
}

MatrixXd CalculateJacobian(const VectorXd& x_state) {

	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

  float denom = px*px + py*py;
	//check division by zero
  if (denom == 0.0f) {
    cout << "Division by zero!" << endl;
    return Hj;
  }
	
	//compute the Jacobian matrix
  float denom_sq = sqrt(denom);
  float denom_32 = denom*denom*denom;
  Hj << px/denom_sq, py/denom_sq, 0, 0,
    -py/denom, px/denom, 0, 0,
    py*(vx*py - vy*px)/denom_32, px*(vy*px - vx*py)/denom_32, px/denom_sq, py/denom_sq;

	return Hj;
}
