#include "fuser/fuser.hpp"

#include <cmath>

namespace fuser {

Fuser::Fuser(const FuserConfig& fuser_config) : config_(fuser_config) {
  g_lidar_ = [](const Eigen::VectorXd& x) -> Eigen::VectorXd {
    Eigen::MatrixXd C{
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
    };
    return C * x;
  };

  g_radar_ = [](const Eigen::VectorXd& x) -> Eigen::VectorXd {
    const double rho = std::sqrt(x(0) * x(0) + x(1) * x(1));
    const double phi = std::atan2(x(1), x(0));
    const double rho_dot = (x(0) * x(2) + x(1) * x(3)) / rho;

    Eigen::VectorXd g_x{{rho, phi, rho_dot}};
    return g_x;
  };
}

void Fuser::SetObject(const Object& object) { object_ = object; }

const Object& Fuser::GetObject() const { return object_; }

void Fuser::ProcessMeasurement(const Measurement& measurement) {
  const double dt = std::chrono::duration_cast<std::chrono::duration<double>>(
                        measurement.GetTimestamp() - object_.timestamp)
                        .count();

  PrepareKalmanFilter(dt, measurement);

  kalman_filter_.Predict(object_.state, object_.covariance_matrix);
  kalman_filter_.Update(object_.state, object_.covariance_matrix,
                        measurement.GetData());

  object_.timestamp = measurement.GetTimestamp();
}

void Fuser::PrepareKalmanFilter(const double dt,
                                const Measurement& measurement) {
  Eigen::MatrixXd A{
      {1.0, 0.0, dt, 0.0},
      {0.0, 1.0, 0.0, dt},
      {0.0, 0.0, 1.0, 0.0},
      {0.0, 0.0, 0.0, 1.0},
  };

  kalman_filter_.SetStateTransitionMatrix(std::move(A));

  const double acc_x = config_.GetMaximumAcceleration()(0);
  const double acc_y = config_.GetMaximumAcceleration()(1);
  const auto dt_2 = dt * dt;
  const auto dt_3 = dt_2 * dt;
  const auto dt_4 = dt_3 * dt;

  Eigen::MatrixXd W{
      {0.25 * dt_4 * acc_x, 0.0, 0.5 * dt_3 * acc_x, 0.0},
      {0.0, 0.25 * dt_4 * acc_y, 0.0, 0.5 * dt_3 * acc_y},
      {0.5 * dt_3 * acc_x, 0.0, dt_2 * acc_x, 0.0},
      {0.0, 0.5 * dt_3 * acc_y, 0.0, dt_2 * acc_y},
  };

  kalman_filter_.SetProcessNoiseCovarianceMatrix(std::move(W));

  switch (measurement.GetSensorType()) {
    case Measurement::SensorType::LIDAR:
      kalman_filter_.SetMeasurementFunction(g_lidar_);
      kalman_filter_.SetMeasurementMatrix(GetLidarMeasurementMatrix());
      kalman_filter_.SetMeasurementNoiseCovarianceMatrix(
          config_.GetLidarNoiseMatrix());
      break;
    case Measurement::SensorType::RADAR:
      kalman_filter_.SetMeasurementFunction(g_radar_);
      kalman_filter_.SetMeasurementMatrix(GetRadarMeasurementMatrix());
      kalman_filter_.SetMeasurementNoiseCovarianceMatrix(
          config_.GetRadarNoiseMatrix());
      break;
    default:
      break;
  }
}

Eigen::MatrixXd Fuser::GetLidarMeasurementMatrix() const {
  Eigen::MatrixXd C_lidar{
      {1.0, 0.0, 0.0, 0.0},
      {0.0, 1.0, 0.0, 0.0},
  };
  return C_lidar;
}

Eigen::MatrixXd Fuser::GetRadarMeasurementMatrix() const {
  const auto& x = object_.state;
  const double c_1 = x(0) * x(0) + x(1) + x(1);
  const double c_2 = std::sqrt(c_1);
  const double c_3 = c_1 * c_2;

  Eigen::MatrixXd C_radar{
      {x(0) / c_2, x(1) / c_2, 0.0, 0.0},
      {-x(1) / c_1, x(0) / c_1, 0.0, 0.0},
      {x(1) * (x(2) * x(1) - x(3) * x(0)) / c_3,
       x(0) * (x(0) * x(3) - x(1) * x(2)) / c_3, x(0) / c_2, x(1) / c_2},
  };
  return C_radar;
}

}  // namespace fuser