#include "Dense"
#include <iostream>
#include<cmath>
#include "LidarTracker.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

LidarTracker::LidarTracker() 
{
	is_initialized_ = false;
	previous_timestamp_ = 0;

	// 4D state vector, we don't know yet the values of the x state
	kf_.x_ = VectorXd(4);

	// State covariance matrix. Assumes we know the approximate position
	// and don't know the speed of the pedestrian 
	kf_.P_ = MatrixXd(4, 4);
	kf_.P_ << 1, 0, 0, 0,
			  0, 1, 0, 0,
			  0, 0, 1000, 0,
			  0, 0, 0, 1000;

	// Measurement covariance matrix
	kf_.V_ = MatrixXd(2, 2);
	kf_.V_ << 0.0225, 0,
			  0, 0.0225;

	// Measurement matrix
	kf_.C_ = MatrixXd(2, 4);
	kf_.C_ << 1, 0, 0, 0,
			  0, 1, 0, 0;

	// The initial state transition matrix A_
	kf_.A_ = MatrixXd(4, 4);
	kf_.A_ << 1, 0, 1, 0,
			  0, 1, 0, 1,
			  0, 0, 1, 0,
			  0, 0, 0, 1;

	// Set the acceleration noise components
	noise_ax = 5;
	noise_ay = 5;
}

LidarTracker::~LidarTracker()
{

}

// Process a single measurement
void LidarTracker::ProcessMeasurement(const Measurement &measurement)
{
	// Initialize Kalman Filter if not initialized
	if (!is_initialized_) 
	{
		cout << "Kalman Filter Initialization" << endl;

		// Set the state with the initial location and zero velocity
		kf_.x_ << measurement.raw_measurements_[0], measurement.raw_measurements_[1], 0, 0;

		previous_timestamp_ = measurement.timestamp_;
		is_initialized_ = true;
		return;
	}

	// Compute the time elapsed between the current and previous measurements
	// dt - expressed in seconds
	float dt = (measurement.timestamp_ - previous_timestamp_) / 1000000.0;	
	previous_timestamp_ = measurement.timestamp_;
	
	// Update A matrix with current dt
	kf_.A_ << 1, 0, dt, 0,
			  0, 1, 0, dt,
			  0, 0, 1, 0,
			  0, 0, 0, 1;

	// Update process covariance matrix W with current dt
	kf_.W_ = MatrixXd(4, 4);
	kf_.W_ << pow(dt, 4.0) / 4 * noise_ax, 0, pow(dt, 3.0) / 2 * noise_ax, 0,
			  0, pow(dt, 4.0) / 4 * noise_ay, 0, pow(dt, 3.0) / 2 * noise_ay,
			  pow(dt, 3.0) / 2 * noise_ax, 0, dt*dt*noise_ax, 0,
			  0, pow(dt, 3.0) / 2 * noise_ay, 0, dt*dt*noise_ay;
	
	// Call the Kalman Filter Predict() function
	kf_.Predict();
	
	// Call the Kalman Filter Update() function
	// with the most recent raw measurements_
	kf_.Update(measurement.raw_measurements_);
	
	std::cout << "x: \n" << kf_.x_ << std::endl;
	std::cout << "P: \n" << kf_.P_ << std::endl << std::endl;
}