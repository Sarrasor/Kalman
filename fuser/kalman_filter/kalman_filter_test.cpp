#include "fuser/kalman_filter/kalman_filter.hpp"

#include <gtest/gtest.h>

namespace fuser {

TEST(KalmanFilterTest, NoiselessPredictTest) {
  const double dt = 0.1;
  Eigen::MatrixXd A{
      {1.0, 0.0, dt, 0.0},
      {0.0, 1.0, 0.0, dt},
      {0.0, 0.0, 1.0, 0.0},
      {0.0, 0.0, 0.0, 1.0},
  };

  Eigen::VectorXd x{{0.0, 0.0, 1.0, 0.0}};
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(x.size(), x.size());
  Eigen::MatrixXd W = Eigen::MatrixXd::Zero(x.size(), x.size());

  Eigen::VectorXd x_true = A * x;
  Eigen::MatrixXd P_true = A * P * A.transpose();

  KalmanFilter kalman_filter;

  kalman_filter.SetStateTransitionMatrix(std::move(A));
  kalman_filter.SetProcessNoiseCovarianceMatrix(std::move(W));

  kalman_filter.Predict(x, P);

  EXPECT_TRUE(x_true.isApprox(x));
  EXPECT_TRUE(P_true.isApprox(P));
}

TEST(KalmanFilterTest, PredictTest) {
  const double dt = 0.1;
  Eigen::MatrixXd A{
      {1.0, 0.0, dt, 0.0},
      {0.0, 1.0, 0.0, dt},
      {0.0, 0.0, 1.0, 0.0},
      {0.0, 0.0, 0.0, 1.0},
  };

  Eigen::VectorXd x{{1.0, -1.0, 1.0, -1.0}};
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(x.size(), x.size());
  Eigen::MatrixXd W = Eigen::MatrixXd::Zero(x.size(), x.size());
  for (int i = 0; i < W.cols(); ++i) {
    W(i, i) = 0.5 * i;
  }

  Eigen::VectorXd x_true = A * x;
  Eigen::MatrixXd P_true = A * P * A.transpose() + W;

  KalmanFilter kalman_filter;

  kalman_filter.SetStateTransitionMatrix(std::move(A));
  kalman_filter.SetProcessNoiseCovarianceMatrix(std::move(W));

  kalman_filter.Predict(x, P);

  EXPECT_TRUE(x_true.isApprox(x));
  EXPECT_TRUE(P_true.isApprox(P));
}

TEST(KalmanFilterTest, NoiselessUpdateTest) {
  Eigen::MatrixXd C{
      {1.0, 0.0, 0.0, 0.0},
      {0.0, 1.0, 0.0, 0.0},
  };

  auto g = [](const Eigen::VectorXd& x) -> Eigen::VectorXd {
    Eigen::MatrixXd C{
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
    };
    return C * x;
  };

  Eigen::VectorXd x{{1.0, 2.0, 1.0, -1.0}};
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(x.size(), x.size());

  Eigen::VectorXd y{{5.0, 10.0}};
  Eigen::MatrixXd V = Eigen::MatrixXd::Zero(y.size(), y.size());

  Eigen::VectorXd x_true{{5.0, 10.0, 1.0, -1.0}};
  Eigen::MatrixXd P_true = P;
  P_true(0, 0) = 0.0;
  P_true(1, 1) = 0.0;

  KalmanFilter kalman_filter;

  kalman_filter.SetMeasurementMatrix(std::move(C));
  kalman_filter.SetMeasurementFunction(g);
  kalman_filter.SetMeasurementNoiseCovarianceMatrix(std::move(V));

  kalman_filter.Update(x, P, y);

  EXPECT_TRUE(x_true.isApprox(x));
  EXPECT_TRUE(P_true.isApprox(P));
}


TEST(KalmanFilterTest, UpdateTest) {
  Eigen::MatrixXd C{
      {1.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 1.0},
  };

  auto g = [](const Eigen::VectorXd& x) -> Eigen::VectorXd {
    Eigen::MatrixXd C{
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 1.0},
    };
    return C * x;
  };

  Eigen::VectorXd x{{1.0, 2.0, 3.0, 4.0}};
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(x.size(), x.size());

  Eigen::VectorXd y{{2.0, 6.0}};
  Eigen::MatrixXd V = Eigen::MatrixXd::Identity(y.size(), y.size());

  Eigen::VectorXd x_true{{1.5, 2.0, 3.0, 5.0}};
  Eigen::MatrixXd P_true = P;
  P_true(0, 0) *= 0.5;
  P_true(3, 3) *= 0.5;

  KalmanFilter kalman_filter;

  kalman_filter.SetMeasurementMatrix(std::move(C));
  kalman_filter.SetMeasurementFunction(g);
  kalman_filter.SetMeasurementNoiseCovarianceMatrix(std::move(V));

  kalman_filter.Update(x, P, y);

  EXPECT_TRUE(x_true.isApprox(x));
  EXPECT_TRUE(P_true.isApprox(P));
}

}  // namespace fuser