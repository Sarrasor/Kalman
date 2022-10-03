#include "fuser/kalman_filter/kalman_filter.hpp"

namespace fuser {
void KalmanFilter::SetStateTransitionMatrix(Eigen::MatrixXd A) {
  A_ = std::move(A);
}

void KalmanFilter::SetProcessNoiseCovarianceMatrix(Eigen::MatrixXd W) {
  W_ = std::move(W);
}

void KalmanFilter::SetMeasurementMatrix(Eigen::MatrixXd C) {
  C_ = std::move(C);
}

void KalmanFilter::SetMeasurementNoiseCovarianceMatrix(Eigen::MatrixXd V) {
  V_ = std::move(V);
}

void KalmanFilter::SetMeasurementFunction(
    Eigen::VectorXd (*g)(const Eigen::VectorXd&)) {
  g_ = g;
}

void KalmanFilter::Predict(Eigen::VectorXd& x, Eigen::MatrixXd& P) const {
  x = A_ * x;
  P = A_ * P * A_.transpose() + W_;
}

void KalmanFilter::Update(Eigen::VectorXd& x, Eigen::MatrixXd& P,
                          const Eigen::VectorXd& y) const {
  auto K = P * C_.transpose() * (C_ * P * C_.transpose() + V_).inverse();
  x += K * (y - g_(x));

  auto I = Eigen::MatrixXd::Identity(x.size(), x.size());
  P = (I - K * C_) * P;
}
}  // namespace fuser