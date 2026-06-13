// Copyright (c) 2024, DDDMobileRobot. BSD-3-Clause.
//
// Persistence layer for STDescManager. Upstream STD has no on-disk format;
// LIO-SAM mapping needs to dump the full descriptor DB at saveMap time and
// sc_global_init / std_global_init needs to load it at startup.
//
// File layout (little-endian, no padding):
//   uint32  magic = 'D','S','T','D' (0x44535444)
//   uint32  version = 1
//   uint32  num_frames        // == plane_cloud_vec_.size()
//   uint32  num_descriptors   // total descriptors across all frames
//   for each descriptor:
//     uint32 frame_id
//     7 * 3 doubles (side_length, angle, center, vertex_A/B/C, vertex_attached)
//   for each frame:
//     uint32 num_plane_points
//     plane_points (each: 3 doubles xyz + 3 doubles normal + 1 float curvature)
//
// Round-trip is bit-exact (no quantization). At ~50 STD/frame and ~1k
// keyframes a typical DB lands at a few MB — still fits memory comfortably.

#pragma once

#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "dddnav_std_descriptor/STDesc.h"

namespace dddnav_std_descriptor
{

constexpr uint32_t kStdDbMagic = 0x44535444u;  // 'DSTD' little-endian
constexpr uint32_t kStdDbVersion = 1u;

inline bool saveStdDatabase(const STDescManager & mgr, const std::string & path)
{
  std::ofstream f(path, std::ios::binary);
  if (!f) {
    std::cerr << "[std_db_io] cannot open for write: " << path << std::endl;
    return false;
  }

  // Flatten descriptors out of the hash table: order is irrelevant for
  // SearchLoop (it rebuilds the hash on load) so we walk frame_id in order
  // for deterministic file content.
  std::vector<const STDesc *> all;
  for (const auto & kv : mgr.data_base_)
    for (const auto & d : kv.second)
      all.push_back(&d);

  const uint32_t num_frames = static_cast<uint32_t>(mgr.plane_cloud_vec_.size());
  const uint32_t num_desc   = static_cast<uint32_t>(all.size());

  f.write(reinterpret_cast<const char *>(&kStdDbMagic),   sizeof(uint32_t));
  f.write(reinterpret_cast<const char *>(&kStdDbVersion), sizeof(uint32_t));
  f.write(reinterpret_cast<const char *>(&num_frames),    sizeof(uint32_t));
  f.write(reinterpret_cast<const char *>(&num_desc),      sizeof(uint32_t));

  for (const STDesc * d : all) {
    f.write(reinterpret_cast<const char *>(&d->frame_id_), sizeof(uint32_t));
    auto wv = [&](const Eigen::Vector3d & v) {
      f.write(reinterpret_cast<const char *>(v.data()), 3 * sizeof(double));
    };
    wv(d->side_length_);
    wv(d->angle_);
    wv(d->center_);
    wv(d->vertex_A_);
    wv(d->vertex_B_);
    wv(d->vertex_C_);
    wv(d->vertex_attached_);
  }

  for (uint32_t i = 0; i < num_frames; ++i) {
    const auto & pc = mgr.plane_cloud_vec_[i];
    const uint32_t n = pc ? static_cast<uint32_t>(pc->size()) : 0u;
    f.write(reinterpret_cast<const char *>(&n), sizeof(uint32_t));
    for (uint32_t j = 0; j < n; ++j) {
      const auto & p = pc->points[j];
      double xyz[3]    = {p.x, p.y, p.z};
      double nrm[3]    = {p.normal_x, p.normal_y, p.normal_z};
      float  curv      = p.curvature;
      f.write(reinterpret_cast<const char *>(xyz),   3 * sizeof(double));
      f.write(reinterpret_cast<const char *>(nrm),   3 * sizeof(double));
      f.write(reinterpret_cast<const char *>(&curv), sizeof(float));
    }
  }
  return f.good();
}

inline bool loadStdDatabase(STDescManager & mgr, const std::string & path)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    std::cerr << "[std_db_io] cannot open for read: " << path << std::endl;
    return false;
  }

  uint32_t magic, version, num_frames, num_desc;
  f.read(reinterpret_cast<char *>(&magic),      sizeof(uint32_t));
  f.read(reinterpret_cast<char *>(&version),    sizeof(uint32_t));
  f.read(reinterpret_cast<char *>(&num_frames), sizeof(uint32_t));
  f.read(reinterpret_cast<char *>(&num_desc),   sizeof(uint32_t));
  if (!f || magic != kStdDbMagic || version != kStdDbVersion) {
    std::cerr << "[std_db_io] header mismatch: magic=" << std::hex << magic
              << " version=" << std::dec << version << std::endl;
    return false;
  }

  std::vector<STDesc> flat(num_desc);
  for (uint32_t i = 0; i < num_desc; ++i) {
    STDesc & d = flat[i];
    f.read(reinterpret_cast<char *>(&d.frame_id_), sizeof(uint32_t));
    auto rv = [&](Eigen::Vector3d & v) {
      f.read(reinterpret_cast<char *>(v.data()), 3 * sizeof(double));
    };
    rv(d.side_length_);
    rv(d.angle_);
    rv(d.center_);
    rv(d.vertex_A_);
    rv(d.vertex_B_);
    rv(d.vertex_C_);
    rv(d.vertex_attached_);
  }
  // AddSTDescs hashes correctly into data_base_ — re-use it instead of
  // duplicating the hash logic.
  mgr.AddSTDescs(flat);
  // current_frame_id_ must move past everything we just loaded so newly
  // ingested keyframes don't collide with stored ones.
  unsigned int max_fid = 0;
  for (const auto & d : flat) max_fid = std::max(max_fid, d.frame_id_);
  mgr.current_frame_id_ = max_fid + 1;

  mgr.plane_cloud_vec_.clear();
  mgr.plane_cloud_vec_.reserve(num_frames);
  for (uint32_t i = 0; i < num_frames; ++i) {
    uint32_t n;
    f.read(reinterpret_cast<char *>(&n), sizeof(uint32_t));
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc(
        new pcl::PointCloud<pcl::PointXYZINormal>());
    pc->reserve(n);
    for (uint32_t j = 0; j < n; ++j) {
      double xyz[3], nrm[3];
      float  curv;
      f.read(reinterpret_cast<char *>(xyz),   3 * sizeof(double));
      f.read(reinterpret_cast<char *>(nrm),   3 * sizeof(double));
      f.read(reinterpret_cast<char *>(&curv), sizeof(float));
      pcl::PointXYZINormal p;
      p.x = xyz[0]; p.y = xyz[1]; p.z = xyz[2];
      p.normal_x = nrm[0]; p.normal_y = nrm[1]; p.normal_z = nrm[2];
      p.curvature = curv;
      pc->push_back(p);
    }
    mgr.plane_cloud_vec_.push_back(pc);
  }
  return f.good() || f.eof();
}

}  // namespace dddnav_std_descriptor
