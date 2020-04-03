#include "KalmanFilter.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;



// Note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in,
									  MatrixXd &A_in, MatrixXd &C_in,
									  MatrixXd &V_in, MatrixXd &W_in)
{
  x_ = x_in;
  P_ = P_in;
  A_ = A_in;
  C_ = C_in;
  V_ = V_in;
  W_ = W_in;
}

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

VectorXd g(const VectorXd x)
{
	VectorXd g_of_x(3);
	
	float px = x(0);
	float py = x(1);
	float vx = x(2);
	float vy = x(3);

	float c1 = px * px + py * py;
	if(fabs(c1) < 0.0001 || fabs(px) < 0.0001)
	{
		std::cout << " g() - Error - Division by Zero" << std::endl;
		g_of_x << 0.0, 0.0, 0.0;
		return g_of_x;
	}

	float rho = sqrt(c1);
	float phi = atan2(py, px);
	float rho_dot = (px*vx + py*vy) / rho;

	g_of_x << rho, phi, rho_dot;
	return g_of_x; 
}

void KalmanFilter::UpdateEKF(const VectorXd &y)
{
  MatrixXd Ct = C_.transpose();
  MatrixXd Y_ = C_ * P_ * Ct + V_;
  MatrixXd K_ = P_ * Ct * Y_.inverse();

  // We are using g(x_) here to represent nonlinearity
  VectorXd z = y - g(x_);

  // Normalize angle
  // Comment this and check what will happen
  while (z(1) > M_PI) z(1) -= 2 * M_PI;
  while (z(1) < -M_PI) z(1) += 2 * M_PI;

  x_ = x_ + K_ * z;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K_ * C_) * P_;

}