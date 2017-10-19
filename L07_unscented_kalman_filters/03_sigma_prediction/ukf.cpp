#include <iostream>
#include "ukf.h"

UKF::UKF() {
  //TODO Auto-generated constructor stub
  Init();
}

UKF::~UKF() {
  //TODO Auto-generated destructor stub
}

  void UKF::Init() {

}


  /*******************************************************************************
   * Programming assignment functions: 
   *******************************************************************************/

  void UKF::SigmaPointPrediction(MatrixXd* Xsig_out) {

  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;

  //create example sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
  Xsig_aug <<
    5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
    1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
    2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
    0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
    0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
    0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
    0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

  double delta_t = 0.1; //time diff in sec
  /*******************************************************************************
   * Student part begin
   ******************************************************************************/
  double p_x, p_y, v, psi, psi_dot, n_a, n_psi, px_term, py_term;

  //predict sigma points
  for (size_t col = 0; col < (size_t)2*n_aug+1; col++) {
    p_x = Xsig_aug(0,col);
    p_y = Xsig_aug(1,col);
    v   = Xsig_aug(2,col);
    psi = Xsig_aug(3,col);
    psi_dot = Xsig_aug(4,col);
    //avoid division by zero
    px_term = (Xsig_aug(4,col) < 0.0001) ? v*std::cos(psi)*delta_t :
                                           (v/psi_dot)*(std::sin(psi + psi_dot*delta_t) - std::sin(psi));
    py_term = (Xsig_aug(4,col) < 0.0001) ? v*std::sin(psi)*delta_t :
                                           (v/psi_dot)*(std::cos(psi) - std::cos(psi + psi_dot*delta_t));
    n_a = Xsig_aug(5,col);
    n_psi = Xsig_aug(6,col);

    Xsig_pred(0, col) = p_x + px_term +
                        std::cos(psi)*n_a*delta_t*delta_t/2;
    Xsig_pred(1, col) = p_y + py_term +
                        std::sin(psi)*n_a*delta_t*delta_t/2;
    Xsig_pred(2, col) = v + delta_t*n_a;
    Xsig_pred(3, col) = psi + psi_dot*delta_t + n_psi*delta_t*delta_t/2;
    Xsig_pred(4, col) = psi_dot + delta_t*n_psi;
  }

  /*******************************************************************************
   * Student part end
   ******************************************************************************/

  //print result
  std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

  //write result
  *Xsig_out = Xsig_pred;

}
