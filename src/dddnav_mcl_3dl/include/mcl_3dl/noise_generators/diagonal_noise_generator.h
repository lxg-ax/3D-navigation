// Copyright (c) 2019, the mcl_3dl authors
// SPDX-License-Identifier: BSD-3-Clause

#ifndef MCL_3DL_NOISE_GENERATORS_DIAGONAL_NOISE_GENERATOR_H
#define MCL_3DL_NOISE_GENERATORS_DIAGONAL_NOISE_GENERATOR_H

#include <random>
#include <vector>

#include <mcl_3dl/noise_generator_base.h>

namespace mcl_3dl
{
template <typename FLT_TYPE>
class DiagonalNoiseGenerator : public NoiseGeneratorBase<FLT_TYPE>
{
public:
  using Parent = NoiseGeneratorBase<FLT_TYPE>;

  template <typename T>
  DiagonalNoiseGenerator(const T& mean, const T& sigma)
  {
    Parent::setMean(mean);
    setSigma(sigma);
  }

  template <typename T>
  void setSigma(const T& sigma)
  {
    sigma_.resize(sigma.size());
    for (size_t i = 0; i < sigma.size(); ++i)
    {
      sigma_[i] = sigma[i];
    }
  }

  template <typename RANDOM_ENGINE>
  std::vector<FLT_TYPE> operator()(RANDOM_ENGINE& engine) const
  {
    std::vector<FLT_TYPE> noise(sigma_.size());
    for (size_t i = 0; i < sigma_.size(); i++)
    {
      std::normal_distribution<FLT_TYPE> nd(Parent::mean_[i], sigma_[i]);
      noise[i] = nd(engine);
    }
    return noise;
  }

protected:
  std::vector<FLT_TYPE> sigma_;
};

}  // namespace mcl_3dl

#endif  // MCL_3DL_NOISE_GENERATORS_DIAGONAL_NOISE_GENERATOR_H
