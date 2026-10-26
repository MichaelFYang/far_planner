#ifndef BOUNDARY_HANDLER_INTERSECTION_H
#define BOUNDARY_HANDLER_INTERSECTION_H

#include <algorithm>
#include <cmath>

namespace POLYOPS {

struct Point2f {
    float x, y;
};

inline bool onSegment(Point2f p, Point2f q, Point2f r) {
    return q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
           q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y);
}

inline int orientation(Point2f p, Point2f q, Point2f r) {
    const double value = (q.y - p.y) * (r.x - q.x) -
                         (q.x - p.x) * (r.y - q.y);
    if (std::abs(value) < 1e-7) return 0;
    return value > 0 ? 1 : 2;
}

inline bool doIntersect(Point2f p1, Point2f q1, Point2f p2, Point2f q2) {
    const int o1 = orientation(p1, q1, p2);
    const int o2 = orientation(p1, q1, q2);
    const int o3 = orientation(p2, q2, p1);
    const int o4 = orientation(p2, q2, q1);

    if (o1 != o2 && o3 != o4) return true;
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;
    return false;
}

}  // namespace POLYOPS
#endif  // BOUNDARY_HANDLER_INTERSECTION_H
