#include <Eigen/Dense>

namespace fuser {

class KalmanFilter {
 public:
  KalmanFilter() = default;

  void SetStateTransitionMatrix(Eigen::MatrixXd A);
  void SetProcessNoiseCovarianceMatrix(Eigen::MatrixXd W);
  void SetMeasurementMatrix(Eigen::MatrixXd C);
  void SetMeasurementNoiseCovarianceMatrix(Eigen::MatrixXd V);
  void SetMeasurementFunction(Eigen::VectorXd (*g)(const Eigen::VectorXd&));

  void Predict(Eigen::VectorXd& x, Eigen::MatrixXd& P) const;
  void Update(Eigen::VectorXd& x, Eigen::MatrixXd& P,
              const Eigen::VectorXd& y) const;

 private:
  // State transition matrix
  Eigen::MatrixXd A_;

  // Process noise covariance matrix
  Eigen::MatrixXd W_;

  // Measurement matrix
  Eigen::MatrixXd C_;

  // Measurement noise covariance matrix
  Eigen::MatrixXd V_;

  // Measurement function
  Eigen::VectorXd (*g_)(const Eigen::VectorXd&) = nullptr;
};

}  // namespace fuser