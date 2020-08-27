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

MatrixXd Tools::CalculateJacobian(const VectorXd& x_) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj = MatrixXd::Zero(3,4);

  // check division by zero
  if ((x_(0) < 0.0001) && (x_(1) < 0.0001))
  {
      std::cout << "Divison by 0" << std::endl;
      return Hj;
  }
  // compute the Jacobian matrix
  double rho_2 = x_(0)*x_(0) + x_(1)*x_(1);
  double rho = sqrt(rho_2);
  double d_rho = (x_(2)*x_(1) - x_(3)*x_(0)) / (rho*rho_2);
  double d_rh0 = (x_(3)*x_(0) - x_(2)*x_(1)) / (rho*rho_2);

  Hj << x_(0)/rho   , x_(1)/rho  , 0.0      , 0.0      ,
        -x_(1)/rho_2, x_(0)/rho_2, 0.0      , 0.0      ,
        x_(1)*d_rho , x_(0)*d_rh0, x_(0)/rho, x_(1)/rho;

  return Hj;
}
