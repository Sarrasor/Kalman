#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <nlohmann/json.hpp>
#include <string>

namespace fuser {

class Measurement {
 public:
  enum class SensorType {
    UNDEFINED = 0,
    GROUND_TRUTH = 1,
    LIDAR = 2,
    RADAR = 3,
  };

  explicit Measurement(const nlohmann::json& json_measurement);

  const std::chrono::microseconds& GetTimestamp() const;
  const SensorType& GetSensorType() const;
  std::string GetSensorTypeString() const;
  const Eigen::VectorXd& GetData() const;

 private:
  friend auto operator<<(std::ostream& os, const Measurement& m)
      -> std::ostream& {
    Eigen::IOFormat fmt(4, 0, ", ", "\n", "\t", "");
    return os << "---Measurement---\n"
              << "Timestamp: " << m.GetTimestamp().count()
              << "\nSensor type: " << m.GetSensorTypeString() << "\nData:\n"
              << m.GetData().format(fmt) << "\n-----------------\n";
  }

  std::chrono::microseconds timestamp_;
  SensorType sensor_type_;
  std::string sensor_type_string_;
  Eigen::VectorXd data_;
};

}  // namespace fuser