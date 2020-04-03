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

    // Initialize Kalman filter
    // @param x_in Initial state
    // @param P_in Initial state covariance
    // @param A_in Transition matrix
    // @param C_in Measurement matrix
    // @param V_in Measurement covariance matrix
    // @param W_in Process covariance matrix
	void Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &A_in,
            MatrixXd &C_in, MatrixXd &V_in, MatrixXd &W_in);

    // Predict the state x' and the state covariance P'
	void Predict();

	// Update the prediction using data from Lidar
	// @param y Data from Lidar
	void Update(const VectorXd &y);

	// Update the prediction using data from Radar
	// @param y Data from Radar
	void UpdateEKF(const Eigen::VectorXd &y);
};

#endif // KALMAN_FILTER_H_