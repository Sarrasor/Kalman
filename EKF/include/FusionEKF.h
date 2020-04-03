#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Dense"
#include "KalmanFilter.h"
#include "Measurement.h"
#include "Tools.h"

class FusionEKF 
{
 public:
  
  // Constructor
  FusionEKF();

  // Destructor
  virtual ~FusionEKF();

  // Run the whole flow of the Kalman Filter from here.
  void ProcessMeasurement(const Measurement &measurement);

  // Kalman Filter update and prediction math lives in here.
  KalmanFilter ekf_;

 private:
  // Check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // Previous timestamp
  long long previous_timestamp_;

  // Tool object used to compute Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd V_laser_;
  Eigen::MatrixXd V_radar_;
  Eigen::MatrixXd C_laser_;
  Eigen::MatrixXd Cj_;

  // Acceleration noise components
  float noise_ax;
  float noise_ay;
};

#endif // FusionEKF_H_
