#include "fuser/object/object.hpp"

#include <limits>

namespace fuser {

Object::Object() : timestamp(0), state(4), covariance_matrix(4, 4) {
  state.setZero();
  covariance_matrix.setZero();
  for (int i = 0; i < covariance_matrix.rows(); ++i) {
    covariance_matrix(i, i) = std::numeric_limits<double>::max();
  }
}

Object::Object(const nlohmann::json& json_object)
    : timestamp(json_object.at("timestamp").get<int64_t>()),
      covariance_matrix(4, 4) {
  auto vec = json_object.at("state").get<std::vector<double>>();
  state = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vec.data(), vec.size());

  auto vec_std = json_object.at("state_std").get<std::vector<double>>();
  covariance_matrix.setZero();
  for (int i = 0; i < covariance_matrix.rows(); ++i) {
    covariance_matrix(i, i) = vec_std[i] * vec_std[i];
  }
}

}  // namespace fuser