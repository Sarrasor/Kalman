#include "FusionEKF.h"
#include <iostream>
#include "Dense"
#include "Tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

// Constructor
FusionEKF::FusionEKF() 
{
  is_initialized_ = false;
  previous_timestamp_ = 0;

  // 4D state vector, we don't know yet the values of the x state
  ekf_.x_ = VectorXd(4);

  // The process noise covariance matrix W
  ekf_.W_ = MatrixXd(4, 4);
  // The state transition matrix A
  ekf_.A_ = MatrixXd(4, 4);

  // Initializing matrices
  V_laser_ = MatrixXd(2, 2);
  V_radar_ = MatrixXd(3, 3);
  C_laser_ = MatrixXd(2, 4);
  Cj_ = MatrixXd(3, 4);

  // Measurement covariance matrix - laser
  V_laser_ << 0.0225, 0,
              0, 0.0225;

  // Measurement covariance matrix - radar
  V_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // Measurement matrix - laser
  C_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // Measurement matrix - radar
  Cj_ << 0, 0, 0, 0,
         0, 0, 0, 0,
         0, 0, 0, 0;

  // Set the acceleration noise components
  noise_ax = 9;
  noise_ay = 9;
}

// Destructor.
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const Measurement &measurement)
 {
  // Initialisation
  if (!is_initialized_) 
  {
    if (measurement.sensor_type_ == Measurement::RADAR) 
    {
      // Convert rho and phi to px and py
      float rho = measurement.raw_measurements_[0];
      float phi = measurement.raw_measurements_[1]; 

      float px = rho * cos(phi);
      float py = rho * sin(phi);
      ekf_.x_ << px, py, 0, 0;

      // Covariance for Radar. We don't know velocity as precise to use it
      ekf_.P_ = MatrixXd(4, 4);
      ekf_.P_ << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;
    }
    else if (measurement.sensor_type_ == Measurement::LASER) 
    {
      ekf_.x_ << measurement.raw_measurements_[0], measurement.raw_measurements_[1], 0, 0;

      // Covariance for Laser. We don't know velocity at all
      ekf_.P_ = MatrixXd(4, 4);
      ekf_.P_ << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;
    }

    previous_timestamp_ = measurement.timestamp_;

    // Done initializing, no need to predict or update
    is_initialized_ = true;

    std::cout << "Initialized" << std::endl; 
    return;
  }

  // Calculate dt
  float dt = (measurement.timestamp_ - previous_timestamp_) / 1000000.0;  
  previous_timestamp_ = measurement.timestamp_;

  // Update A matrix with current dt
  ekf_.A_ << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;

  // Update process covariance matrix W with current dt
  
  ekf_.W_ << pow(dt, 4.0) / 4 * noise_ax, 0, pow(dt, 3.0) / 2 * noise_ax, 0,
        0, pow(dt, 4.0) / 4 * noise_ay, 0, pow(dt, 3.0) / 2 * noise_ay,
        pow(dt, 3.0) / 2 * noise_ax, 0, dt*dt*noise_ax, 0,
        0, pow(dt, 3.0) / 2 * noise_ay, 0, dt*dt*noise_ay;

  // Prediction
  ekf_.Predict();

  // Update
  if (measurement.sensor_type_ == Measurement::RADAR) 
  {
    // Radar updates
    ekf_.V_ = V_radar_;

    // We calculate new Jacobian at each time step
    Cj_ = tools.CalculateRadarJacobian(ekf_.x_);
    ekf_.C_ = Cj_;

    ekf_.UpdateEKF(measurement.raw_measurements_);
  } 
  else
  {
    // Laser updates
    ekf_.V_ = V_laser_;
    ekf_.C_ = C_laser_;

    ekf_.Update(measurement.raw_measurements_);
  }

  // Print the output
  cout << "x: \n" << ekf_.x_ << endl;
  cout << "P: \n" << ekf_.P_ << endl << endl;
}
