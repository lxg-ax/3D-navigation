// Copyright (c) 2020, the mcl_3dl authors
// SPDX-License-Identifier: BSD-3-Clause

#ifndef MCL_3DL_FILTER_VEC3_H
#define MCL_3DL_FILTER_VEC3_H

#include <array>

#include <mcl_3dl/filter.h>
#include <mcl_3dl/vec3.h>

namespace mcl_3dl
{
class FilterVec3
{
private:
  Vec3 x_;
  std::array<Filter, 3> f_;

public:
  inline FilterVec3(Filter::type_t type, const Vec3& time_const, const Vec3& out0, const bool angle = false)
    : x_(out0)
    , f_(  // clang-format off
          {
              Filter(type, time_const[0], out0[0], angle),
              Filter(type, time_const[1], out0[1], angle),
              Filter(type, time_const[2], out0[2], angle),
          }
        )  // clang-format on
  {
  }
  inline void set(const Vec3& out0)
  {
    x_ = out0;
    for (int i = 0; i < 3; i++)
      f_[i].set(out0[i]);
  }
  inline Vec3 in(const Vec3& in)
  {
    for (int i = 0; i < 3; i++)
      x_[i] = f_[i].in(in[i]);
    return x_;
  }
  inline Vec3 get() const
  {
    return x_;
  }
};
}  // namespace mcl_3dl

#endif  // MCL_3DL_FILTER_VEC3_H
