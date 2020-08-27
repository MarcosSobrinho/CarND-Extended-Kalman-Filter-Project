#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */

    VectorXd rmse = VectorXd::Zero(4);

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0) {
    std::cout << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj = MatrixXd::Zero(3,4);
  // recover state parameters
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  // TODO: YOUR CODE HERE

  // check division by zero
  if (px == 0.0f && py == 0.0f)
  {
      std::cout << "Divison by 0" << std::endl;
      return Hj;
  }
  // compute the Jacobian matrix
  double rho_2 = px*px + py*py;
  double rho = sqrt(rho_2);
  double d_rho = (vx*py - vy*px) / (rho*rho_2);
  double d_rh0 = (vy*px - vx*py) / (rho*rho_2);

  Hj << px/rho   , py/rho   , 0.0   , 0.0   ,
        -py/rho_2, px/rho_2 , 0.0   , 0.0   ,
        py*d_rho , px*d_rh0 , px/rho, py/rho;

  return Hj;
}
