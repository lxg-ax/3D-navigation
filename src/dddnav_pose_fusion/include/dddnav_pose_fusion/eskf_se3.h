// Copyright (c) 2024, DDDMobileRobot. BSD-3-Clause.
//
// Loose-coupling ESKF on SE(3) for fusing a high-rate body-frame odometry
// (FAST-LIO output, ~100 Hz) with a low-rate global pose measurement
// (MCL 3DL output, ~5 Hz).
//
// State variables
//   nominal:  T_mb = (p_mb, q_mb)            position 3, quaternion 4
//   error:    delta_x = (delta_p, delta_theta) in R^6, right-perturbation
//             delta_theta defined so that q_true = q * Exp(delta_theta)
//
// Why an error-state KF, not a vanilla EKF on (p, q)?
//   * The quaternion has a unit-norm constraint that breaks linear KF
//     covariance updates; carrying covariance on the 3-vector tangent
//     keeps it minimal and well-defined.
//   * Right-perturbation makes the measurement Jacobian cleanly identity
//     when both the measurement and state live in the same map frame.
//
// Process model (predict)
//   We treat FAST-LIO as a relative-pose source. Between two consecutive
//   odometry messages we form the body-frame delta:
//       T_b1_b2 = T_o_b1^-1 * T_o_b2
//   and compose it onto the nominal state:
//       T_mb_new = T_mb * T_b1_b2
//   Process noise Q is added on the tangent (translation x 3, rotation x 3).
//
// Measurement model (update)
//   z = (p_meas, q_meas), residual on tangent:
//       innovation_p     = p_meas - p_mb
//       innovation_theta = Log( q_mb^-1 * q_meas )
//   H = I_6, joseph-form covariance update for symmetry preservation.
//
// Reset / injection
//   After update we inject the error into the nominal state and zero the
//   error mean. The covariance is left as-is (this is the standard ESKF
//   trick — the second-order correction term is small enough to ignore for
//   this application).

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace dddnav_pose_fusion
{

// Right-perturbation Jacobian helpers ---------------------------------------
// Hat operator: R^3 -> so(3). Standard cross-product matrix.
Eigen::Matrix3d hat(const Eigen::Vector3d & v);

// SO(3) exponential, Rodrigues formula. Maps R^3 axis-angle -> rotation.
Eigen::Quaterniond expSO3(const Eigen::Vector3d & w);

// SO(3) logarithm: rotation -> R^3 axis-angle. Returns zero for identity.
Eigen::Vector3d logSO3(const Eigen::Quaterniond & q);

class EskfSE3
{
public:
  // 6x6 covariance: rows/cols [position(3), rotation(3)]
  using Cov = Eigen::Matrix<double, 6, 6>;
  using Vec6 = Eigen::Matrix<double, 6, 1>;

  EskfSE3();

  // Set the nominal state and an initial covariance. Typically called once
  // when the first MCL pose arrives (or from an /initial_3d_pose).
  void initialize(
    const Eigen::Vector3d & p,
    const Eigen::Quaterniond & q,
    const Cov & P);

  // Whether initialize() has been called.
  bool initialized() const { return initialized_; }

  // Predict step using a body-frame relative transform (T_b_prev_b_now).
  //   delta_p, delta_q : the body-frame delta (typically computed by the
  //                      caller from two consecutive FAST-LIO odometry
  //                      messages so that the bridge stays generic).
  //   Q                : 6x6 process noise covariance contributed by this
  //                      step (caller scales by dt — see the node).
  void predict(
    const Eigen::Vector3d & delta_p,
    const Eigen::Quaterniond & delta_q,
    const Cov & Q);

  // Measurement update with a global pose (map frame).
  //   R : 6x6 measurement covariance.
  // Mahalanobis-gated: returns false and skips the update if the squared
  // innovation exceeds gate_chi2 (chi^2_6, 99% ≈ 16.81). Pass <=0 to disable
  // the gate.
  bool update(
    const Eigen::Vector3d & p_meas,
    const Eigen::Quaterniond & q_meas,
    const Cov & R,
    double gate_chi2 = 16.81);

  // Accessors
  const Eigen::Vector3d & position() const { return p_; }
  const Eigen::Quaterniond & orientation() const { return q_; }
  const Cov & covariance() const { return P_; }

private:
  Eigen::Vector3d p_;
  Eigen::Quaterniond q_;
  Cov P_;
  bool initialized_;

  // Inject error state into nominal state and reset error mean to zero.
  void inject(const Vec6 & delta);
};

}  // namespace dddnav_pose_fusion
