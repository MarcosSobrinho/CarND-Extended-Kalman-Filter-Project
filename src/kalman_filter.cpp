#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_*x_;
  P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z, const MatrixXd& H_l, const MatrixXd& R_) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  Eigen::Vector2d y;
  MatrixXd S = MatrixXd(2,2);
  MatrixXd K = MatrixXd(4,2);

  y = z - H_l*x_;
  S = H_l*P_*H_l.transpose() + R_;
  K = P_*H_l.transpose()*S.inverse();

  x_ = x_ + K*y;
  P_ = (MatrixXd::Identity(4,4) - K*H_l)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z, const MatrixXd& H_j, const MatrixXd& R_) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  Eigen::Vector3d y;
  Eigen::Matrix<double, 3, 3> S;
  Eigen::Matrix<double, 4, 3> K;

  y(0) = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  y(1) = atan2(x_(1), x_(0));
  y(2) = (x_(0)*x_(2) + x_(1)*x_(3)) / y(0);

  y = z - y;

  S = H_j*P_*H_j.transpose() + R_;
  K = P_*H_j.transpose()*S.inverse();

  x_ = x_ + K*y;
  P_ = (MatrixXd::Identity(4,4) - K*H_j)*P_;
}
