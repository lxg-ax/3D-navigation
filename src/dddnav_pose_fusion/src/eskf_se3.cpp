// Copyright (c) 2024, DDDMobileRobot. BSD-3-Clause.
#include "dddnav_pose_fusion/eskf_se3.h"

namespace dddnav_pose_fusion
{

Eigen::Matrix3d hat(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d S;
  S <<     0, -v.z(),  v.y(),
       v.z(),     0, -v.x(),
      -v.y(),  v.x(),     0;
  return S;
}

Eigen::Quaterniond expSO3(const Eigen::Vector3d & w)
{
  // Rodrigues. For very small angle, fall back to first-order to avoid
  // dividing by ||w||. Threshold chosen so that sin(theta)/theta error stays
  // below ~1e-12.
  const double theta = w.norm();
  if (theta < 1e-9) {
    Eigen::Quaterniond q(1.0, 0.5 * w.x(), 0.5 * w.y(), 0.5 * w.z());
    q.normalize();
    return q;
  }
  const Eigen::Vector3d axis = w / theta;
  const double half = 0.5 * theta;
  const double s = std::sin(half);
  return Eigen::Quaterniond(std::cos(half), s * axis.x(), s * axis.y(), s * axis.z());
}

Eigen::Vector3d logSO3(const Eigen::Quaterniond & q_in)
{
  // Use a normalised, w >= 0 quaternion so the log returns the shorter arc.
  Eigen::Quaterniond q = q_in.normalized();
  if (q.w() < 0) {
    q.coeffs() = -q.coeffs();
  }
  const Eigen::Vector3d v(q.x(), q.y(), q.z());
  const double n = v.norm();
  if (n < 1e-9) {
    return 2.0 * v;  // first-order: theta ≈ 2 * sin(theta/2)
  }
  const double theta = 2.0 * std::atan2(n, q.w());
  return v * (theta / n);
}

EskfSE3::EskfSE3()
: p_(Eigen::Vector3d::Zero()),
  q_(Eigen::Quaterniond::Identity()),
  P_(Cov::Zero()),
  initialized_(false)
{
}

void EskfSE3::initialize(
  const Eigen::Vector3d & p,
  const Eigen::Quaterniond & q,
  const Cov & P)
{
  p_ = p;
  q_ = q.normalized();
  P_ = P;
  initialized_ = true;
}

void EskfSE3::predict(
  const Eigen::Vector3d & delta_p,
  const Eigen::Quaterniond & delta_q,
  const Cov & Q)
{
  // Compose body-frame relative transform onto the nominal state.
  //   p_new = p + R * delta_p
  //   q_new = q * delta_q
  const Eigen::Matrix3d R = q_.toRotationMatrix();
  p_ = p_ + R * delta_p;
  q_ = (q_ * delta_q).normalized();

  // State-transition Jacobian on the right-perturbation tangent (block form):
  //   [ I    -R * hat(delta_p) ]
  //   [ 0    Exp(-delta_theta).T  ]
  // For small motion deltas (which is what one FAST-LIO step gives us at
  // 100 Hz) the rotation block is very close to identity, so we use the
  // common simplification F = I + small terms and add a non-trivial
  // translation-rotation cross-term so that orientation uncertainty
  // propagates into position correctly.
  Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
  F.block<3, 3>(0, 3) = -R * hat(delta_p);

  P_ = F * P_ * F.transpose() + Q;
}

bool EskfSE3::update(
  const Eigen::Vector3d & p_meas,
  const Eigen::Quaterniond & q_meas,
  const Cov & R_cov,
  double gate_chi2)
{
  // Innovation on the tangent (right-perturbation):
  //   y_p     = p_meas - p
  //   y_theta = log(q^-1 * q_meas)
  Vec6 y;
  y.head<3>() = p_meas - p_;
  y.tail<3>() = logSO3(q_.conjugate() * q_meas.normalized());

  // H = I (measurement is the state itself).
  // S = H P H^T + R = P + R
  Eigen::Matrix<double, 6, 6> S = P_ + R_cov;

  if (gate_chi2 > 0.0) {
    const double m = y.transpose() * S.ldlt().solve(y);
    if (m > gate_chi2) {
      return false;
    }
  }

  // Kalman gain. ldlt().solve is numerically stabler than explicit inverse.
  Eigen::Matrix<double, 6, 6> K = P_ * S.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());

  // Error-state mean and Joseph-form covariance.
  Vec6 dx = K * y;
  Eigen::Matrix<double, 6, 6> I_KH = Eigen::Matrix<double, 6, 6>::Identity() - K;  // H = I
  P_ = I_KH * P_ * I_KH.transpose() + K * R_cov * K.transpose();

  inject(dx);
  return true;
}

void EskfSE3::inject(const Vec6 & dx)
{
  p_ += dx.head<3>();
  q_ = (q_ * expSO3(dx.tail<3>())).normalized();
}

}  // namespace dddnav_pose_fusion
