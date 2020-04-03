#include "Tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
	// Initialise rmse vector
 	VectorXd rmse(4);
 	rmse << 0, 0, 0, 0;

 	// Check for errors
	if (estimations.size() == 0 || estimations.size() != ground_truth.size())
	{
		std::cout << "CalculateRMSE() - Error - Invalid data" << std::endl;
		return rmse;
	}

	// Calculate the sum of residuals
	for (int i = 0; i < estimations.size(); ++i)
	{
		VectorXd residual = estimations[i] - ground_truth[i];

		residual = residual.array()*residual.array();
		rmse += residual;
	}

	// Normalize the sum
	rmse /= estimations.size();
	rmse = rmse.array().sqrt();

	return rmse;
}

MatrixXd Tools::CalculateRadarJacobian(const VectorXd& x_state)
{
	MatrixXd Cj(3,4);
	
	// Extract state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	// Pre-compute a set of terms to avoid repeated calculation
	float c1 = px * px + py * py;
	float c2 = sqrt(c1);
	float c3 = (c1 * c2);

	// Check division by zero
	if(fabs(c1) < 0.0001)
	{
		std::cout << "CalculateRadarJacobian() - Error - Division by Zero" << std::endl;
		return Cj;
	}

	// Compute the Radar Jacobian matrix
	Cj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

	return Cj;
}
