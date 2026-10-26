#ifndef BOUNDARY_HANDLER_POINT_STRUCT_H
#define BOUNDARY_HANDLER_POINT_STRUCT_H

#include <cmath>

struct Point3D {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  Point3D() = default;
  Point3D(float x_value, float y_value, float z_value)
      : x(x_value), y(y_value), z(z_value) {}

  Point3D operator+(const Point3D& other) const {
    return Point3D(x + other.x, y + other.y, z + other.z);
  }

  Point3D operator-(const Point3D& other) const {
    return Point3D(x - other.x, y - other.y, z - other.z);
  }

  Point3D operator-() const { return Point3D(-x, -y, -z); }

  float operator*(const Point3D& other) const {
    return x * other.x + y * other.y + z * other.z;
  }

  Point3D operator*(float factor) const {
    return Point3D(x * factor, y * factor, z * factor);
  }

  float norm_flat() const { return std::hypot(x, y); }

  Point3D normalize_flat() const {
    const float length = norm_flat();
    return length > 1e-7f ? Point3D(x / length, y / length, 0.0f) : Point3D();
  }
};

#endif  // BOUNDARY_HANDLER_POINT_STRUCT_H
