#include "fuser/object/object.hpp"

#include <gtest/gtest.h>

namespace fuser {

TEST(ObjectTest, DefaultInitTest) {
  Eigen::VectorXd state{{0.0, 0.0, 0.0, 0.0}};
  auto max = std::numeric_limits<double>::max();
  Eigen::MatrixXd covariance_matrix{{max, 0.0, 0.0, 0.0},
                                    {0.0, max, 0.0, 0.0},
                                    {0.0, 0.0, max, 0.0},
                                    {0.0, 0.0, 0.0, max}};
  Object object;

  EXPECT_TRUE(state.isApprox(object.state));
  EXPECT_TRUE(covariance_matrix.isApprox(object.covariance_matrix));
}

TEST(ObjectTest, JsonInitTest) {
  nlohmann::json json_object;
  json_object["timestamp"] = 100;
  json_object["state"] = {1.0, 2.0, 3.0, 4.0};
  json_object["state_std"] = {0.1, 0.2, 0.3, 0.4};
  Eigen::VectorXd state{{1.0, 2.0, 3.0, 4.0}};
  Eigen::MatrixXd covariance_matrix{{0.1 * 0.1, 0.0, 0.0, 0.0},
                                    {0.0, 0.2 * 0.2, 0.0, 0.0},
                                    {0.0, 0.0, 0.3 * 0.3, 0.0},
                                    {0.0, 0.0, 0.0, 0.4 * 0.4}};

  Object object(json_object);

  EXPECT_EQ(object.timestamp.count(), 100);
  EXPECT_TRUE(state.isApprox(object.state));
  EXPECT_TRUE(covariance_matrix.isApprox(object.covariance_matrix));
}

}  // namespace fuser