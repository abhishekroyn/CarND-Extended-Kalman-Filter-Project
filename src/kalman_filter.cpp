#include "kalman_filter.h"
#include "FusionEKF.h"
#include <math.h>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

// predict the state by using Kalman Filter equations
void KalmanFilter::Predict() {

  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

// update the state by using Kalman Filter equations
void KalmanFilter::Update(const VectorXd &z) {

  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // new state
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

// update the state by using Extended Kalman Filter equations
void KalmanFilter::UpdateEKF(const VectorXd &z) {

  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float c1 = px * px + py * py;
  float rho = sqrt(c1);
  float theta = atan2(py, px);
  float rho_dot = (px * vx + py * vy) / rho;

  // check division by zero
  if (fabs(c1) < 0.0001) {
    std::cout << "UpdateEKF () - Error - Division by Zero" << std::endl;
  }

  // current estimated values after conversion from polar to cartesian coordinates
  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta, rho_dot;

  VectorXd y = z - z_pred;

  // keep Kalman filter angle value small between the range -pi and pi
  while ((y(1) < -M_PI) || (y(1) > M_PI)) {
    if (y(1) < -M_PI) {
      y(1) += 2*M_PI;
    } else {
      y(1) -= 2*M_PI;
    }
  }

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new state
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
