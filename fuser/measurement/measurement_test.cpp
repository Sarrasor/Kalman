#include "fuser/measurement/measurement.hpp"

#include <gtest/gtest.h>

namespace fuser {

TEST(MeasurementTest, InitTest) {
  nlohmann::json json_measurement;
  json_measurement["timestamp"] = 100;
  json_measurement["sensor_type"] = 2;
  json_measurement["data"] = {1.0, 2.0, 3.0};
  Eigen::VectorXd data{{1.0, 2.0, 3.0}};

  Measurement measurement(json_measurement);

  EXPECT_EQ(measurement.GetTimestamp(), std::chrono::microseconds(100));
  EXPECT_EQ(measurement.GetSensorType(), Measurement::SensorType::LIDAR);
  EXPECT_EQ(measurement.GetSensorTypeString(), "LIDAR");
  EXPECT_TRUE(data.isApprox(measurement.GetData()));
}

}  // namespace fuser