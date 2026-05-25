// Copyright (c) 2024, DDDMobileRobot. BSD-3-Clause.
//
// Unit tests for the SE(3) ESKF math (Rodrigues / log-SO(3) / predict /
// update). These run under ament's gtest harness and gate the dddnav_*
// regression CI — they catch the silent symmetry / positive-definiteness
// failures that only show up as drift in long bag replays otherwise.

#include <cmath>

#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include "dddnav_pose_fusion/eskf_se3.h"

using dddnav_pose_fusion::EskfSE3;
using dddnav_pose_fusion::expSO3;
using dddnav_pose_fusion::hat;
using dddnav_pose_fusion::logSO3;

namespace
{

bool approxEq(double a, double b, double eps = 1e-9)
{
  return std::fabs(a - b) < eps;
}

bool quatApproxEq(const Eigen::Quaterniond & a, const Eigen::Quaterniond & b,
                  double eps = 1e-6)
{
  // Quaternions q and -q describe the same rotation, so compare by absolute
  // dot product instead of element-wise.
  const double d = std::fabs(
    a.w() * b.w() + a.x() * b.x() + a.y() * b.y() + a.z() * b.z());
  return std::fabs(1.0 - d) < eps;
}

bool symmetric(const EskfSE3::Cov & P, double eps = 1e-9)
{
  return (P - P.transpose()).cwiseAbs().maxCoeff() < eps;
}

bool positiveSemiDefinite(const EskfSE3::Cov & P, double eps = -1e-9)
{
  Eigen::SelfAdjointEigenSolver<EskfSE3::Cov> es(P);
  return es.info() == Eigen::Success
      && es.eigenvalues().minCoeff() >= eps;
}

EskfSE3::Cov diagCov(double pos, double rot)
{
  EskfSE3::Cov P = EskfSE3::Cov::Zero();
  P.diagonal() << pos, pos, pos, rot, rot, rot;
  return P;
}

}  // namespace

// expSO3 is the right-inverse of logSO3 for any rotation. We sweep a few
// representative axes / magnitudes including small-angle (where the
// Rodrigues formula falls back to first-order).
TEST(SO3, ExpLogRoundtrip)
{
  const std::vector<Eigen::Vector3d> axes = {
    {0.0, 0.0, 1.0},
    {1.0, 1.0, 0.0},
    {0.5, -0.7, 0.2},
    {1.0, 2.0, 3.0},
  };
  const std::vector<double> mags = {1e-12, 1e-6, 1e-3, 0.5, 1.0, M_PI - 1e-3};
  for (const auto & axis : axes) {
    const Eigen::Vector3d u = axis.normalized();
    for (double m : mags) {
      const Eigen::Vector3d w = m * u;
      const auto q = expSO3(w);
      const auto w_back = logSO3(q);
      EXPECT_TRUE(approxEq((w_back - w).norm(), 0.0, 1e-6))
        << "axis=(" << u.transpose() << ") mag=" << m
        << " back=(" << w_back.transpose() << ")";
    }
  }
}

// hat() is skew-symmetric and produces the standard cross product.
TEST(SO3, HatIsSkewAndCross)
{
  const Eigen::Vector3d a(1.2, -0.3, 0.7);
  const Eigen::Vector3d b(0.4,  0.9, -0.5);
  const Eigen::Matrix3d S = hat(a);
  EXPECT_TRUE((S + S.transpose()).cwiseAbs().maxCoeff() < 1e-12);
  EXPECT_TRUE((S * b - a.cross(b)).cwiseAbs().maxCoeff() < 1e-12);
}

// Predict followed by update keeps covariance symmetric and PSD.
TEST(EskfSE3, PredictUpdateKeepsCovarianceSymmetricAndPSD)
{
  EskfSE3 f;
  f.initialize(Eigen::Vector3d(1.0, 2.0, 0.0),
               Eigen::Quaterniond::Identity(),
               diagCov(0.25, 0.04));

  const auto Q = diagCov(0.01 * 0.01, 0.005 * 0.005);
  const auto R = diagCov(0.10 * 0.10, 0.05 * 0.05);

  // Drive the filter through a curved trajectory.
  const double dyaw = 0.05;
  const Eigen::Vector3d step(0.1, 0.0, 0.0);
  for (int i = 0; i < 50; ++i) {
    const Eigen::Quaterniond dq(Eigen::AngleAxisd(dyaw, Eigen::Vector3d::UnitZ()));
    f.predict(step, dq, Q);
    EXPECT_TRUE(symmetric(f.covariance(), 1e-9))
      << "predict: P not symmetric at i=" << i;
    EXPECT_TRUE(positiveSemiDefinite(f.covariance()))
      << "predict: P not PSD at i=" << i;

    if (i % 5 == 0) {
      // Use the current state as the measurement so the innovation is
      // small but nonzero (after we add a perturbation). This stresses
      // the Joseph form without tripping the Mahalanobis gate.
      const Eigen::Vector3d p_meas = f.position()
                                   + Eigen::Vector3d(0.01, -0.02, 0.0);
      const auto q_meas =
        f.orientation()
        * Eigen::Quaterniond(Eigen::AngleAxisd(0.005,
                                               Eigen::Vector3d::UnitZ()));
      const bool accepted = f.update(p_meas, q_meas, R);
      EXPECT_TRUE(accepted) << "update rejected at i=" << i;
      EXPECT_TRUE(symmetric(f.covariance(), 1e-9))
        << "update: P not symmetric at i=" << i;
      EXPECT_TRUE(positiveSemiDefinite(f.covariance()))
        << "update: P not PSD at i=" << i;
    }
  }
}

// An update with the same pose as the state should be a no-op (innovation
// = 0), and the covariance should shrink toward the measurement noise.
TEST(EskfSE3, ZeroInnovationShrinksCovariance)
{
  EskfSE3 f;
  const auto P0 = diagCov(0.5 * 0.5, 0.3 * 0.3);
  f.initialize(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), P0);
  const auto R = diagCov(0.05 * 0.05, 0.02 * 0.02);
  ASSERT_TRUE(f.update(Eigen::Vector3d::Zero(),
                       Eigen::Quaterniond::Identity(), R));

  const auto & P1 = f.covariance();
  for (int i = 0; i < 6; ++i) {
    EXPECT_LT(P1(i, i), P0(i, i))
      << "diag " << i << " did not shrink: " << P0(i, i)
      << " -> " << P1(i, i);
  }
  // State should be untouched (innovation was zero).
  EXPECT_TRUE(approxEq(f.position().norm(), 0.0, 1e-9));
  EXPECT_TRUE(quatApproxEq(f.orientation(), Eigen::Quaterniond::Identity()));
}

// Mahalanobis gate rejects an obviously wild measurement. We pick an
// innovation magnitude well above the chi^2_6 99% threshold.
TEST(EskfSE3, MahalanobisGateRejects)
{
  EskfSE3 f;
  const auto P = diagCov(0.01, 0.01);
  f.initialize(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), P);
  const auto R = diagCov(0.01, 0.01);

  // 10-sigma off in position: definitely outside chi^2_6 = 16.81.
  const Eigen::Vector3d p_meas(10.0, 0.0, 0.0);
  EXPECT_FALSE(f.update(p_meas, Eigen::Quaterniond::Identity(), R, 16.81));
  // ...and is accepted with the gate disabled.
  EXPECT_TRUE(f.update(p_meas, Eigen::Quaterniond::Identity(), R, 0.0));
}

// Predict with zero motion + zero process noise must leave both the state
// and covariance untouched. Catches accidental side-effects in the state
// transition Jacobian.
TEST(EskfSE3, IdleStepIsIdempotent)
{
  EskfSE3 f;
  const auto P0 = diagCov(0.4, 0.2);
  f.initialize(Eigen::Vector3d(3.0, -1.5, 0.2),
               Eigen::Quaterniond(Eigen::AngleAxisd(
                 0.7, Eigen::Vector3d::UnitZ())),
               P0);

  const auto p0 = f.position();
  const auto q0 = f.orientation();
  f.predict(Eigen::Vector3d::Zero(),
            Eigen::Quaterniond::Identity(),
            EskfSE3::Cov::Zero());
  EXPECT_TRUE(approxEq((f.position() - p0).norm(), 0.0, 1e-12));
  EXPECT_TRUE(quatApproxEq(f.orientation(), q0));
  EXPECT_TRUE((f.covariance() - P0).cwiseAbs().maxCoeff() < 1e-12);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
