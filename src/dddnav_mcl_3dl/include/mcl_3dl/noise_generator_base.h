// Copyright (c) 2019, the mcl_3dl authors
// SPDX-License-Identifier: BSD-3-Clause

#ifndef MCL_3DL_NOISE_GENERATOR_BASE_H
#define MCL_3DL_NOISE_GENERATOR_BASE_H

#include <vector>

namespace mcl_3dl
{
template <typename FLT_TYPE>
class NoiseGeneratorBase
{
public:
  virtual ~NoiseGeneratorBase()
  {
  }

  template <typename T>
  void setMean(const T& mean)
  {
    mean_.resize(mean.size());
    for (size_t i = 0; i < mean.size(); ++i)
    {
      mean_[i] = mean[i];
    }
  }
  const std::vector<FLT_TYPE>& getMean() const
  {
    return mean_;
  }
  size_t getDimension() const
  {
    return mean_.size();
  }

protected:
  std::vector<FLT_TYPE> mean_;
};


}  // namespace mcl_3dl

#endif  // MCL_3DL_NOISE_GENERATOR_BASE_H
