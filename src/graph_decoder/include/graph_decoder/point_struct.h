#ifndef GRAPH_DECODER_POINT_STRUCT_H
#define GRAPH_DECODER_POINT_STRUCT_H

// Decoder graph positions and surface directions need only Cartesian coordinates.
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

  Point3D operator*(float factor) const {
    return Point3D(x * factor, y * factor, z * factor);
  }
};

#endif  // GRAPH_DECODER_POINT_STRUCT_H
