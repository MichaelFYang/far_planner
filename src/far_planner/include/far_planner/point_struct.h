#ifndef POINT_STRUCT_H
#define POINT_STRUCT_H

#define EPSILON 1e-7f

#include <math.h>
#include <vector>
#include <utility>
#include <iostream>
#include <stdlib.h>
#include <deque>
#include <unordered_map>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/distances.h>
#include <boost/functional/hash.hpp>

// ROS message support
#include <ros/console.h>


typedef pcl::PointXYZI PCLPoint;
typedef pcl::PointCloud<PCLPoint> PointCloud;
typedef pcl::PointCloud<PCLPoint>::Ptr PointCloudPtr;

struct Point3D {
  float x, y, z;
  float intensity;
  Point3D() = default;
  Point3D(float _x, float _y, float _z):\
      x(_x), y(_y), z(_z), intensity(0) {}
  Point3D(float _x, float _y, float _z, float _i):\
      x(_x), y(_y), z(_z), intensity(_i) {}
  Point3D(Eigen::Vector3f vec):\
      x(vec(0)), y(vec(1)), z(vec(2)), intensity(0) {}
  Point3D(Eigen::Vector3d vec):\
      x(vec(0)), y(vec(1)), z(vec(2)), intensity(0) {}
  Point3D(PCLPoint pt):\
      x(pt.x), y(pt.y), z(pt.z), intensity(pt.intensity) {}

  bool operator ==(const Point3D& pt) const
  {
    return fabs(x - pt.x) < EPSILON &&
           fabs(y - pt.y) < EPSILON &&
           fabs(z - pt.z) < EPSILON;
  }

  bool operator !=(const Point3D& pt) const
  {
    return fabs(x - pt.x) > EPSILON ||
           fabs(y - pt.y) > EPSILON ||
           fabs(z - pt.z) > EPSILON;
  }

  float operator *(const Point3D& pt) const
  {
      return x * pt.x + y * pt.y + z * pt.z;
  }

  Point3D operator *(const float factor) const
  {
      return Point3D(x*factor, y*factor, z*factor);
  }

  Point3D operator /(const float factor) const
  {
      return Point3D(x/factor, y/factor, z/factor);
  }

  Point3D operator +(const Point3D& pt) const
  {
      return Point3D(x+pt.x, y+pt.y, z+pt.z);
  }
  template <typename Point>
  Point3D operator -(const Point& pt) const
  {
      return Point3D(x-pt.x, y-pt.y, z-pt.z);
  }
  Point3D operator -() const
  {
      return Point3D(-x, -y, -z);
  }
  float norm() const
  {
    return std::hypotf(x, std::hypotf(y,z));
  };

  float norm_flat() const
  {
    return std::hypotf(x, y);
  };

  Point3D normalize() const 
  {
    const float n = std::hypotf(x, std::hypotf(y,z));
    if (n > EPSILON) {
      return Point3D(x/n, y/n, z/n);
    } else {
      // DEBUG
      // ROS_WARN("Point3D: vector normalization fails, vector norm is too small.");
      return Point3D(0,0,0);
    }
  };

  Point3D normalize_flat() const 
  {
    const float n = std::hypotf(x, y);
    if (n > EPSILON) {
      return Point3D(x/n, y/n, 0.0f);
    } else {
      // DEBUG
      // ROS_WARN("Point3D: flat vector normalization fails, vector norm is too small.");
      return Point3D(0,0,0);
    }
  };

  float norm_dot(Point3D p) const
  {
    const float n1 = std::hypotf(x, std::hypotf(y,z));
    const float n2 = std::hypotf(p.x, std::hypotf(p.y,p.z));
    if (n1 < EPSILON || n2 < EPSILON) {
      // DEBUG
      // ROS_WARN("Point3D: vector norm dot fails, vector norm is too small.");
      return 0.f;
    }
    const float dot_value = (x * p.x + y * p.y + z * p.z) / (n1 * n2);
    return std::min(std::max(-1.0f, dot_value), 1.0f);
  };

  float norm_flat_dot(Point3D p) const
  {
    const float n1 = std::hypotf(x, y);
    const float n2 = std::hypotf(p.x, p.y);
    if (n1 < EPSILON || n2 < EPSILON) {
      // DEBUG
      // ROS_WARN("Point3D: flat vector norm dot fails, vector norm is too small.");
      return 0.f;
    }
    const float dot_value = (x * p.x + y * p.y) / (n1 * n2);
    return std::min(std::max(-1.0f, dot_value), 1.0f);
  };

  std::string ToString() const
  {
    return "x = " + std::to_string(x).substr(0,6) + "; " + 
           "y = " + std::to_string(y).substr(0,6) + "; " + 
           "z = " + std::to_string(z).substr(0,6) + " "; 
  };
  
  friend std::ostream& operator<<(std::ostream& os, const Point3D& p)
  {
    os << "["<< p.ToString() << "] ";
    return os;
  }
};

struct point_hash
{
  std::size_t operator() (const Point3D& p) const
  {
    std::size_t seed = 0;
    boost::hash_combine(seed, p.x);
    boost::hash_combine(seed, p.y);
    boost::hash_combine(seed, p.z);
    return seed;
  }
};

struct point_comp
{
  bool operator()(const Point3D& p1, const Point3D& p2) const
  {
    return p1 == p2;
  }
};

struct intensity_comp
{
  bool operator()(const Point3D& p1, const Point3D& p2) const
  {
    return p1.intensity < p2.intensity;
  }
};

#endif