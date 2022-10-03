#include "fuser/measurement/measurement.hpp"

#include <vector>

namespace fuser {

Measurement::Measurement(const nlohmann::json& json_measurement)
    : timestamp_(json_measurement.at("timestamp").get<int64_t>()),
      sensor_type_(static_cast<Measurement::SensorType>(
          json_measurement.at("sensor_type").get<int>())) {
  auto vec = json_measurement.at("data").get<std::vector<double>>();
  data_ = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vec.data(), vec.size());

  switch (sensor_type_) {
    case Measurement::SensorType::UNDEFINED:
      sensor_type_string_ = "UNDEFINED";
      break;
    case Measurement::SensorType::GROUND_TRUTH:
      sensor_type_string_ = "GROUND_TRUTH";
      break;
    case Measurement::SensorType::LIDAR:
      sensor_type_string_ = "LIDAR";
      break;
    case Measurement::SensorType::RADAR:
      sensor_type_string_ = "RADAR";
      break;
  }
}

const std::chrono::microseconds& Measurement::GetTimestamp() const {
  return timestamp_;
}

const Measurement::SensorType& Measurement::GetSensorType() const {
  return sensor_type_;
}

std::string Measurement::GetSensorTypeString() const {
  return sensor_type_string_;
}

const Eigen::VectorXd& Measurement::GetData() const { return data_; }

}  // namespace fuser