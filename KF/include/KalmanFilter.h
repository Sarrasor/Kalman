#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter
{
 public:
 	// State vector
 	VectorXd x_;

 	// State covariance matrix
 	MatrixXd P_;

 	// State transition matrix
 	MatrixXd A_;

 	// Process covariance matrix
 	MatrixXd W_;

 	// Measurement matrix
 	MatrixXd C_;

 	// Measurement covariance matrix
 	MatrixXd V_;

 	// Constructor
 	KalmanFilter();

 	// Destructor
	virtual ~KalmanFilter();

  // Predict the state x' and the state covariance P'
	void Predict();

	// Update the prediction using data y from sensor
	void Update(const VectorXd &y);
};

#endif // KALMAN_FILTER_H_