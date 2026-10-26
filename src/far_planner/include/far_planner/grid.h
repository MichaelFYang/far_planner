#ifndef GRID_UTIL_H
#define GRID_UTIL_H
// SPDX-License-Identifier: MIT

/**
 * @file grid.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a 3D grid
 * @version 0.1
 * @date 2021-01-26
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <cfloat>
#include <vector>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace grid_ns
{
template <typename _T, bool Sparse = false>
class Grid
{
public:
  explicit Grid(const Eigen::Vector3i& size, _T init_value, const Eigen::Vector3d& origin = Eigen::Vector3d(0, 0, 0),
                const Eigen::Vector3d& resolution = Eigen::Vector3d(1, 1, 1), int dimension = 3)
    : init_value_(std::move(init_value))
  {
    origin_ = origin;
    size_ = size;
    resolution_ = resolution;
    resolution_inv_.setZero();
    dimension_ = dimension;

    if (dimension_ < 1 || dimension_ > 3) {
      throw std::invalid_argument("Grid dimension must be between 1 and 3");
    }
    int count = 1;
    for (int i = 0; i < 3; ++i) {
      if (size_(i) <= 0 || count > std::numeric_limits<int>::max() / size_(i)) {
        throw std::length_error("Grid cell count exceeds the supported range");
      }
      count *= size_(i);
    }
    for (int i = 0; i < dimension_; i++)
    {
      if (!std::isfinite(resolution_(i)) || resolution_(i) <= 0.0 ||
          !std::isfinite(1.0 / resolution_(i))) {
        throw std::invalid_argument("Grid resolution must be positive and finite");
      }
      resolution_inv_(i) = 1.0 / resolution_(i);
    }
    cell_number_ = count;
    if (!Sparse) cells_.assign(cell_number_, init_value_);
  }

  virtual ~Grid() = default;

  int GetCellNumber() const
  {
    return cell_number_;
  }

  Eigen::Vector3i GetSize() const
  {
    return size_;
  }

  Eigen::Vector3d GetOrigin() const
  {
    return origin_;
  }

  void SetOrigin(const Eigen::Vector3d& origin)
  {
    origin_ = origin;
  }

  void ReInitGrid(const _T& init_value) 
  {
    init_value_ = init_value;
    if (Sparse) sparse_cells_.clear();
    else std::fill(cells_.begin(), cells_.end(), init_value_);
  }

  void SetResolution(const Eigen::Vector3d& resolution)
  {
    Eigen::Vector3d inverse = Eigen::Vector3d::Zero();
    for (int i = 0; i < dimension_; i++)
    {
      if (!std::isfinite(resolution(i)) || resolution(i) <= 0.0 ||
          !std::isfinite(1.0 / resolution(i))) {
        throw std::invalid_argument("Grid resolution must be positive and finite");
      }
      inverse(i) = 1.0 / resolution(i);
    }
    resolution_ = resolution;
    resolution_inv_ = inverse;
  }

  Eigen::Vector3d GetResolution() const
  {
    return resolution_;
  }

  Eigen::Vector3d GetResolutionInv() const
  {
    return resolution_inv_;
  }

  bool InRange(int x, int y, int z) const
  {
    return InRange(Eigen::Vector3i(x, y, z));
  }

  bool InRange(const Eigen::Vector3i& sub) const
  {
    bool in_range = true;
    for (int i = 0; i < dimension_; i++)
    {
      in_range &= sub(i) >= 0 && sub(i) < size_(i);
    }
    return in_range;
  }

  bool InRange(int ind) const
  {
    return ind >= 0 && ind < cell_number_;
  }

  Eigen::Vector3i Ind2Sub(int ind) const
  {
    return ind2sub_(ind);
  }

  int Sub2Ind(int x, int y, int z) const
  {
    return x + (y * size_.x()) + (z * size_.x() * size_.y());
  }

  int Sub2Ind(const Eigen::Vector3i& sub) const
  {
    // MY_ASSERT(InRange(sub));
    return Sub2Ind(sub.x(), sub.y(), sub.z());
  }

  Eigen::Vector3d Sub2Pos(int x, int y, int z) const
  {
    return Sub2Pos(Eigen::Vector3i(x, y, z));
  }

  Eigen::Vector3d Sub2Pos(const Eigen::Vector3i& sub) const
  {
    Eigen::Vector3d pos(0, 0, 0);
    for (int i = 0; i < dimension_; i++)
    {
      pos(i) = origin_(i) + sub(i) * resolution_(i) + resolution_(i) / 2.0;
    }
    return pos;
  }

  Eigen::Vector3d Ind2Pos(int ind) const
  {
    // MY_ASSERT(InRange(ind));
    return Sub2Pos(Ind2Sub(ind));
  }

  Eigen::Vector3i Pos2Sub(double x, double y, double z) const
  {
    return Pos2Sub(Eigen::Vector3d(x, y, z));
  }

  Eigen::Vector3i Pos2Sub(const Eigen::Vector3d& pos) const
  {
    Eigen::Vector3i sub(0, 0, 0);
    const int limit = std::numeric_limits<int>::max() / 2;
    for (int i = 0; i < dimension_; i++)
    {
      const double offset = pos(i) - origin_(i);
      if (offset >= 0.0) {
        const double scaled = offset * resolution_inv_(i);
        sub(i) = scaled < size_(i) || scaled < limit ?
                 static_cast<int>(scaled) : std::max(limit, size_(i));
        continue;
      }
      if (offset >= -1e-7) {
        sub(i) = 0;  // Preserve tolerance at the lower boundary.
        continue;
      }
      if (!std::isfinite(offset)) {
        sub(i) = -limit;
        continue;
      }
      const double scaled = offset * resolution_inv_(i);
      sub(i) = !std::isfinite(scaled) || scaled <= -limit ?
               -limit : static_cast<int>(std::floor(scaled));
    }
    return sub;
  }

  int Pos2Ind(const Eigen::Vector3d& pos) const
  {
    return Sub2Ind(Pos2Sub(pos));
  }

  _T& GetCell(int x, int y, int z)
  {
    return GetCell(Eigen::Vector3i(x, y, z));
  }

  _T& GetCell(const Eigen::Vector3i& sub)
  {
    return GetCell(Sub2Ind(sub));
  }

  _T& GetCell(int index)
  {
    if (Sparse) {
      const auto found = sparse_cells_.find(index);
      if (found != sparse_cells_.end()) return found->second;
      return sparse_cells_.emplace(index, init_value_).first->second;
    }
    return cells_[index];
  }

  _T GetCellValue(int x, int y, int z) const
  {
    return GetCellValue(Sub2Ind(x, y, z));
  }

  _T GetCellValue(const Eigen::Vector3i& sub) const
  {
    // MY_ASSERT(InRange(sub));
    return GetCellValue(sub.x(), sub.y(), sub.z());
  }

  _T GetCellValue(int index) const
  {
    if (Sparse) {
      const auto found = sparse_cells_.find(index);
      return found == sparse_cells_.end() ? init_value_ : found->second;
    }
    return cells_[index];
  }

  void EraseCell(int index)
  {
    if (Sparse) sparse_cells_.erase(index);
    else cells_[index] = init_value_;
  }

  std::size_t MaterializedCellCount() const
  {
    return Sparse ? sparse_cells_.size() : cells_.size();
  }

  void SetCellValue(int x, int y, int z, _T value)
  {
    int index = Sub2Ind(x, y, z);
    GetCell(index) = value;
  }

  void SetCellValue(const Eigen::Vector3i& sub, _T value)
  {
    // MY_ASSERT(InRange(sub));
    SetCellValue(sub.x(), sub.y(), sub.z(), value);
  }

  void SetCellValue(int index, const _T& value)
  {
    // MY_ASSERT(InRange(index));
    GetCell(index) = value;
  }

  void RayTraceSubs(const Eigen::Vector3i& start_sub, 
                    const Eigen::Vector3i& end_sub,
                    std::vector<Eigen::Vector3i>& subs)
  {
    subs.clear();
    if (!InRange(start_sub)) return;
    const Eigen::Vector3d diff_sub = end_sub.cast<double>() - start_sub.cast<double>();
    const double max_dist = diff_sub.squaredNorm();
    const int step_x = (end_sub.x() > start_sub.x()) - (end_sub.x() < start_sub.x());
    const int step_y = (end_sub.y() > start_sub.y()) - (end_sub.y() < start_sub.y());
    const int step_z = (end_sub.z() > start_sub.z()) - (end_sub.z() < start_sub.z());
    const double t_delta_x = step_x == 0 ? DBL_MAX : 1.0 / std::abs(diff_sub.x());
    const double t_delta_y = step_y == 0 ? DBL_MAX : 1.0 / std::abs(diff_sub.y());
    const double t_delta_z = step_z == 0 ? DBL_MAX : 1.0 / std::abs(diff_sub.z());


    double t_max_x = step_x == 0 ? DBL_MAX : intbound(start_sub.x(), diff_sub.x());
    double t_max_y = step_y == 0 ? DBL_MAX : intbound(start_sub.y(), diff_sub.y());
    double t_max_z = step_z == 0 ? DBL_MAX : intbound(start_sub.z(), diff_sub.z());
    double dist = 0;
    Eigen::Vector3i cur_sub = start_sub;

    while (InRange(cur_sub)) {
      subs.push_back(cur_sub);
      dist = (cur_sub.cast<double>() - start_sub.cast<double>()).squaredNorm();
      if (cur_sub == end_sub || dist > max_dist)
      {
        return;
      }
      if (t_max_x < t_max_y)
      {
        if (t_max_x < t_max_z)
        {
          cur_sub.x() += step_x;
          t_max_x += t_delta_x;
        }
        else
        {
          cur_sub.z() += step_z;
          t_max_z += t_delta_z;
        }
      }
      else
      {
        if (t_max_y < t_max_z)
        {
          cur_sub.y() += step_y;
          t_max_y += t_delta_y;
        }
        else
        {
          cur_sub.z() += step_z;
          t_max_z += t_delta_z;
        }
      }
    }
  }

private:
  Eigen::Vector3d origin_;
  Eigen::Vector3i size_;
  Eigen::Vector3d resolution_;
  Eigen::Vector3d resolution_inv_;
  std::vector<_T> cells_;
  std::unordered_map<int, _T> sparse_cells_;
  _T init_value_;
  int cell_number_;
  int dimension_;

  Eigen::Vector3i ind2sub_(int ind) const
  {
    // MY_ASSERT(InRange(ind));
    Eigen::Vector3i sub;
    sub.z() = ind / (size_.x() * size_.y());
    ind -= (sub.z() * size_.x() * size_.y());
    sub.y() = ind / size_.x();
    sub.x() = ind % size_.x();
    return sub;
  }

  // Math Helper functions
  double mod(const double& value, const double& modulus)
  {
    return std::fmod(std::fmod(value, modulus) + modulus, modulus);
  }
  double intbound(double s, double ds)
  {
    // Find the smallest positive t such that s+t*ds is an integer.
    if (ds < 0)
    {
      return intbound(-s, -ds);
    }
    else
    {
      s = mod(s, 1);
      // problem is now s+t*ds = 1
      return (1 - s) / ds;
    }
  }

};
}  // namespace grid_ns

#endif
