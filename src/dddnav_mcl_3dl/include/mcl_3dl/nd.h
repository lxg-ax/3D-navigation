// Copyright (c) 2016-2017, the mcl_3dl authors
// SPDX-License-Identifier: BSD-3-Clause

#ifndef MCL_3DL_ND_H
#define MCL_3DL_ND_H

#define _USE_MATH_DEFINES
#include <cmath>

#include <Eigen/Core>
#include <Eigen/LU>

namespace mcl_3dl
{
template <typename FLT_TYPE = float>
class NormalLikelihood
{
public:
  explicit NormalLikelihood(const FLT_TYPE sigma)
  {
    a_ = 1.0 / std::sqrt(2.0 * M_PI * sigma * sigma);
    sq2_ = sigma * sigma * 2.0;
  }
  FLT_TYPE operator()(const FLT_TYPE x) const
  {
    return a_ * expf(-x * x / sq2_);
  }

protected:
  FLT_TYPE a_;
  FLT_TYPE sq2_;
};

template <typename FLT_TYPE = float, size_t DIMENSION = 6>
class NormalLikelihoodNd
{
public:
  using Matrix = Eigen::Matrix<FLT_TYPE, DIMENSION, DIMENSION>;
  using Vector = Eigen::Matrix<FLT_TYPE, DIMENSION, 1>;

  explicit NormalLikelihoodNd(const Matrix sigma)
  {
    a_ = 1.0 / (std::pow(2.0 * M_PI, 0.5 * DIMENSION) * std::sqrt(sigma.determinant()));
    sigma_inv_ = sigma.inverse();
  }
  FLT_TYPE operator()(const Vector x) const
  {
    return a_ * std::exp(static_cast<FLT_TYPE>(-0.5 * x.transpose() * sigma_inv_ * x));
  }

protected:
  FLT_TYPE a_;
  Matrix sigma_inv_;
};
}  // namespace mcl_3dl

#endif  // MCL_3DL_ND_H
