#include "kalman_filter.h"
#include "tools.h"
#include <cmath>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

void KalmanFilter::Initialize(const Eigen::VectorXd &x, const Eigen::MatrixXd &P)
{
  x_ = x;
  P_ = P;
  F_ = MatrixXd::Identity(4, 4);
  I = MatrixXd::Identity(4, 4);
}

void KalmanFilter::Predict(float dt)
{
  F_(0, 2) = dt;
  F_(1, 3) = dt;
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z)
{
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;
}

namespace
{

VectorXd h(const MatrixXd &x)
{
  VectorXd v(3);
  v(0) = sqrt(x(0) * x(0) + x(1) * x(1));
  v(1) = atan2(x(1), x(0));
  v(2) = (x(0) * x(2) + x(1) * x(3)) / v(0);
  return v;
}

} // namespace
void KalmanFilter::UpdateEKF(const VectorXd &z)
{

  auto hx = h(x_);
  std::cout << "z  = " << z(0) << ", " << z(1) << ", " << z(2) << std::endl;
  std::cout << "hx = " << hx(0) << ", " << hx(1) << ", " << hx(2) << std::endl;
  VectorXd y = z - hx;
  if (y(1) > M_PI)
  {
    y(1) -= 2 * M_PI;
  }
  if (y(1) < -M_PI)
  {
    y(1) += 2 * M_PI;
  }
  std::cout << "y  = " << y(0) << ", " << y(1) << ", " << y(2) << std::endl;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;
}
