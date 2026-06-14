// Copyright (c) 2019, the mcl_3dl authors
// SPDX-License-Identifier: BSD-3-Clause

#ifndef MCL_3DL_MOTION_PREDICTION_MODEL_BASE_H
#define MCL_3DL_MOTION_PREDICTION_MODEL_BASE_H

#include <memory>

#include <mcl_3dl/state_6dof.h>

namespace mcl_3dl
{
class MotionPredictionModelBase
{
public:
  using Ptr = std::shared_ptr<MotionPredictionModelBase>;

  virtual void setOdoms(const State6DOF& odom_prev, const State6DOF& odom_current, const float time_diff) = 0;
  virtual void predict(State6DOF& s) const = 0;
};
}  // namespace mcl_3dl

#endif  // MCL_3DL_MOTION_PREDICTION_MODEL_BASE_H
