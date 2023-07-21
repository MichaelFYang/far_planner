#ifndef GRID_UTIL_H
#define GRID_UTIL_H

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

namespace grid_ns
{
template <typename _T>
class Grid
{
public:
  explicit Grid(const Eigen::Vector3i& size, _T init_value, const Eigen::Vector3d& origin = Eigen::Vector3d(0, 0, 0),
                const Eigen::Vector3d& resolution = Eigen::Vector3d(1, 1, 1), int dimension = 3)
  {
    // MY_ASSERT(size.x() > 0);
    // MY_ASSERT(size.y() > 0);
    // MY_ASSERT(size.z() > 0);

    origin_ = origin;
    size_ = size;
    resolution_ = resolution;
    dimension_ = dimension;

    for (int i = 0; i < dimension_; i++)
    {
      resolution_inv_(i) = 1.0 / resolution_(i);
    }
    cell_number_ = size_.x() * size_.y() * size_.z();
    cells_.resize(cell_number_), subs_.resize(cell_number_);
    for (int i = 0; i < cell_number_; i++)
    {
      cells_[i] = init_value;
      subs_[i] = ind2sub_(i);
    }
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
    std::fill(cells_.begin(), cells_.end(), init_value);
  }

  void SetResolution(const Eigen::Vector3d& resolution)
  {
    resolution_ = resolution;
    for (int i = 0; i < dimension_; i++)
    {
      resolution_inv_(i) = 1.0 / resolution(i);
    }
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
    // MY_ASSERT(InRange(ind));
    return subs_[ind];
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
    for (int i = 0; i < dimension_; i++)
    {
      sub(i) = pos(i) - origin_(i) > -1e-7 ? static_cast<int>((pos(i) - origin_(i)) * resolution_inv_(i)) : -1;
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
    // MY_ASSERT(InRange(sub));
    int index = Sub2Ind(sub);
    return cells_[index];
  }

  _T& GetCell(int index)
  {
    // MY_ASSERT(InRange(index));
    return cells_[index];
  }

  _T GetCellValue(int x, int y, int z) const
  {
    int index = Sub2Ind(x, y, z);
    return cells_[index];
  }

  _T GetCellValue(const Eigen::Vector3i& sub) const
  {
    // MY_ASSERT(InRange(sub));
    return GetCellValue(sub.x(), sub.y(), sub.z());
  }

  _T GetCellValue(int index) const
  {
    // MY_ASSERT(InRange(index));
    return cells_[index];
  }

  void SetCellValue(int x, int y, int z, _T value)
  {
    int index = Sub2Ind(x, y, z);
    cells_[index] = value;
  }

  void SetCellValue(const Eigen::Vector3i& sub, _T value)
  {
    // MY_ASSERT(InRange(sub));
    SetCellValue(sub.x(), sub.y(), sub.z(), value);
  }

  void SetCellValue(int index, const _T& value)
  {
    // MY_ASSERT(InRange(index));
    cells_[index] = value;
  }

  void RayTraceSubs(const Eigen::Vector3i& start_sub, 
                    const Eigen::Vector3i& end_sub,
                    std::vector<Eigen::Vector3i>& subs)
  {
    subs.clear();
    const Eigen::Vector3i diff_sub = end_sub - start_sub;
    const double max_dist = diff_sub.squaredNorm();
    const int step_x = signum(diff_sub.x());
    const int step_y = signum(diff_sub.y());
    const int step_z = signum(diff_sub.z());
    const double t_delta_x = step_x == 0 ? DBL_MAX : (double)step_x / (double)diff_sub.x();
    const double t_delta_y = step_y == 0 ? DBL_MAX : (double)step_y / (double)diff_sub.y();
    const double t_delta_z = step_z == 0 ? DBL_MAX : (double)step_z / (double)diff_sub.z();


    double t_max_x = step_x == 0 ? DBL_MAX : intbound(start_sub.x(), diff_sub.x());
    double t_max_y = step_y == 0 ? DBL_MAX : intbound(start_sub.y(), diff_sub.y());
    double t_max_z = step_z == 0 ? DBL_MAX : intbound(start_sub.z(), diff_sub.z());
    double dist = 0;
    Eigen::Vector3i cur_sub = start_sub;

    while (InRange(cur_sub)) {
      subs.push_back(cur_sub);
      dist = (cur_sub - start_sub).squaredNorm();
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
  std::vector<Eigen::Vector3i> subs_;
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
  int signum(const int& x)
  {
    return x == 0 ? 0 : x < 0 ? -1 : 1;
  }
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