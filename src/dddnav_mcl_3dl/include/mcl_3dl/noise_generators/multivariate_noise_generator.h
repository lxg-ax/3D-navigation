// Copyright (c) 2019, the mcl_3dl authors
// SPDX-License-Identifier: BSD-3-Clause

#ifndef MCL_3DL_NOISE_GENERATORS_MULTIVARIATE_NOISE_GENERATOR_H
#define MCL_3DL_NOISE_GENERATORS_MULTIVARIATE_NOISE_GENERATOR_H

#include <random>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <mcl_3dl/noise_generator_base.h>

namespace mcl_3dl
{
template <typename FLT_TYPE>
class MultivariateNoiseGenerator : public NoiseGeneratorBase<FLT_TYPE>
{
public:
  using Parent = NoiseGeneratorBase<FLT_TYPE>;
  using Matrix = Eigen::Matrix<FLT_TYPE, Eigen::Dynamic, Eigen::Dynamic>;
  using Vector = Eigen::Matrix<FLT_TYPE, Eigen::Dynamic, 1>;

  template <typename MEAN_TYPE, typename COV_TYPE>
  explicit MultivariateNoiseGenerator(const MEAN_TYPE& mean, const COV_TYPE& covariance)
  {
    Parent::setMean(mean);
    mean_vec_.resize(Parent::mean_.size());
    for (int i = 0; i < mean_vec_.size(); ++i)
    {
      mean_vec_[i] = Parent::mean_[i];
    }
    setCovariance(covariance);
  }

  template <typename COV_TYPE>
  void setCovariance(const COV_TYPE& covariance)
  {
    const size_t dim = Parent::getDimension();
    Matrix cov_matrix(dim, dim);
    for (size_t i = 0; i < dim; ++i)
    {
      for (size_t j = 0; j < dim; ++j)
      {
        cov_matrix(i, j) = covariance[j + i * dim];
      }
    }
    const Eigen::SelfAdjointEigenSolver<Matrix> eigen_solver(cov_matrix);
    norm_transform_ = eigen_solver.eigenvectors() * eigen_solver.eigenvalues().cwiseSqrt().asDiagonal();
  }

  template <typename RANDOM_ENGINE>
  std::vector<FLT_TYPE> operator()(RANDOM_ENGINE& engine) const
  {
    const size_t dim = Parent::getDimension();
    Vector random(dim);
    std::normal_distribution<FLT_TYPE> nd(0.0, 1.0);
    for (size_t j = 0; j < dim; ++j)
    {
      random[j] = nd(engine);
    }
    const Vector sample_noise = mean_vec_ + norm_transform_ * random;
    return std::vector<FLT_TYPE>(sample_noise.data(), sample_noise.data() + dim);
  }

protected:
  Vector mean_vec_;
  Matrix norm_transform_;
};

}  // namespace mcl_3dl

#endif  // MCL_3DL_NOISE_GENERATORS_MULTIVARIATE_NOISE_GENERATOR_H
