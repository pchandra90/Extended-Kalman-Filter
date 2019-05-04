#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &R_radar_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  R_radar_ = R_radar_in;
  Q_ = Q_in;
  long x_size = x_.size();
  I = MatrixXd::Identity(x_size, x_size);
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  CommonUpdate(y, H_, R_);
  return;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * update the state by using Extended Kalman Filter equations
   */
  MatrixXd Hj = tools_.CalculateJacobian(x_);
  VectorXd x_polar = tools_.CartesianToPolar(x_);
  VectorXd y = z - x_polar;
  // make sure anage is in between -pi to +pi
  while(y(1) > M_PI){
  	y(1) -= 2*M_PI;
  }
  
  while(y(1) < -M_PI){
  	y(1) += 2*M_PI;
  }
  CommonUpdate(y, Hj, R_radar_);
  return;
}

void KalmanFilter::CommonUpdate(const VectorXd &y, const MatrixXd &H, const MatrixXd &R) {
  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht + R;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  //new estimate
  x_ = x_ + (K * y);
  P_ = (I - K * H) * P_;
  return;
}
