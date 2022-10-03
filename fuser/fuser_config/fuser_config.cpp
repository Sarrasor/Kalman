#include "fuser/fuser_config/fuser_config.hpp"

namespace fuser {

FuserConfig::FuserConfig(const nlohmann::json& json_config)
    : lidar_noise_matrix_(2, 2), radar_noise_matrix_(3, 3) {
  auto vec = json_config.at("max_acceleration").get<std::vector<double>>();
  max_acceleration_ =
      Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vec.data(), vec.size());

  lidar_noise_matrix_.setZero();
  for (int i = 0; i < lidar_noise_matrix_.rows(); ++i) {
    auto sigma = json_config.at("lidar_noise_std").at(i).get<double>();
    lidar_noise_matrix_(i, i) = sigma * sigma;
  }

  radar_noise_matrix_.setZero();
  for (int i = 0; i < radar_noise_matrix_.rows(); ++i) {
    auto sigma = json_config.at("radar_noise_std").at(i).get<double>();
    radar_noise_matrix_(i, i) = sigma * sigma;
  }
}

const Eigen::VectorXd& FuserConfig::GetMaximumAcceleration() const {
  return max_acceleration_;
}

const Eigen::MatrixXd& FuserConfig::GetLidarNoiseMatrix() const {
  return lidar_noise_matrix_;
}

const Eigen::MatrixXd& FuserConfig::GetRadarNoiseMatrix() const {
  return radar_noise_matrix_;
}

}  // namespace fuser