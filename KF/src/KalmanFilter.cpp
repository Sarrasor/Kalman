#include "KalmanFilter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() 
{
  x_ = A_ * x_;
  P_ = A_ * P_ * A_.transpose() + W_;
}

void KalmanFilter::Update(const VectorXd &y) 
{
  MatrixXd Ct = C_.transpose();
  MatrixXd Y_ = C_ * P_ * Ct + V_;
  MatrixXd K_ = P_ * Ct * Y_.inverse();

  x_ = x_ + K_ * (y - C_ * x_);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K_ * C_) * P_;
}