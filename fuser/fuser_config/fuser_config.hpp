#pragma once

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

namespace fuser {

class FuserConfig {
 public:
  explicit FuserConfig(const nlohmann::json& json_config);

  const Eigen::VectorXd& GetMaximumAcceleration() const;
  const Eigen::MatrixXd& GetLidarNoiseMatrix() const;
  const Eigen::MatrixXd& GetRadarNoiseMatrix() const;

 private:
  Eigen::VectorXd max_acceleration_;
  Eigen::MatrixXd lidar_noise_matrix_;
  Eigen::MatrixXd radar_noise_matrix_;
};

}  // namespace fuser