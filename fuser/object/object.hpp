#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <nlohmann/json.hpp>

namespace fuser {

struct Object {
  Object();
  explicit Object(const nlohmann::json& json_object);

  friend auto operator<<(std::ostream& os, const Object& object)
      -> std::ostream& {
    Eigen::IOFormat fmt(4, 0, ", ", "\n", "\t", "");
    return os << "---Object---\n"
              << "Timestamp: " << object.timestamp.count() << "\nState:\n"
              << object.state.format(fmt) << "\nCovariance:\n"
              << object.covariance_matrix.format(fmt) << "\n------------\n";
  }

  std::chrono::microseconds timestamp;
  Eigen::VectorXd state;
  Eigen::MatrixXd covariance_matrix;
};

}  // namespace fuser