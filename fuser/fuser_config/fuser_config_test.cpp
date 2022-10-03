#include "fuser/fuser_config/fuser_config.hpp"

#include <gtest/gtest.h>

namespace fuser {

TEST(FuserConfigTest, JsonInitTest) {
  nlohmann::json json_config;
  json_config["max_acceleration"] = {2.0, 3.0};
  json_config["lidar_noise_std"] = {1.0, 2.0};
  json_config["radar_noise_std"] = {0.1, 0.2, 0.3};

  Eigen::VectorXd max_acceleration{{2.0, 3.0}};

  Eigen::MatrixXd lidar_noise_matrix{
      {1.0, 0.0},
      {0.0, 4.0},
  };

  Eigen::MatrixXd radar_noise_matrix{
      {0.1 * 0.1, 0.0, 0.0},
      {0.0, 0.2 * 0.2, 0.0},
      {0.0, 0.0, 0.3 * 0.3},
  };

  FuserConfig config(json_config);

  EXPECT_TRUE(max_acceleration.isApprox(config.GetMaximumAcceleration()));
  EXPECT_TRUE(lidar_noise_matrix.isApprox(config.GetLidarNoiseMatrix()));
  EXPECT_TRUE(radar_noise_matrix.isApprox(config.GetRadarNoiseMatrix()));
}

}  // namespace fuser