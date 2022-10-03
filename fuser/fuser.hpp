#pragma once

#include <Eigen/Dense>

#include "fuser/fuser_config/fuser_config.hpp"
#include "fuser/kalman_filter/kalman_filter.hpp"
#include "fuser/measurement/measurement.hpp"
#include "fuser/object/object.hpp"

namespace fuser {

class Fuser {
 public:
  explicit Fuser(const FuserConfig& fuser_config);

  void SetObject(const Object& object);
  const Object& GetObject() const;

  void ProcessMeasurement(const Measurement& measurement);

 private:
  void PrepareKalmanFilter(const double dt, const Measurement& measurement);
  Eigen::MatrixXd GetLidarMeasurementMatrix() const;
  Eigen::MatrixXd GetRadarMeasurementMatrix() const;

  FuserConfig config_;
  Object object_;

  Eigen::VectorXd (*g_lidar_)(const Eigen::VectorXd&);
  Eigen::VectorXd (*g_radar_)(const Eigen::VectorXd&);

  KalmanFilter kalman_filter_;
};

}  // namespace fuser