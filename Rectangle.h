#ifndef RECTANGLE_H
#define RECTANGLE_H

#include "Point.h"
#include <algorithm>
#include <limits>
#include <vector>

template <size_t DIM> class Rectangle {
public:
  Point<DIM> minPoint;
  Point<DIM> maxPoint;

  Rectangle() {
    for (size_t i = 0; i < DIM; ++i) {
      minPoint.coords[i] = std::numeric_limits<double>::max();
      maxPoint.coords[i] = std::numeric_limits<double>::lowest();
    }
  }

  Rectangle(const Point<DIM> &p) : minPoint(p), maxPoint(p) {}

  Rectangle(const Point<DIM> &minP, const Point<DIM> &maxP)
      : minPoint(minP), maxPoint(maxP) {}

  double area() const {
    double a = 1.0;
    for (size_t i = 0; i < DIM; ++i) {
      a *= (maxPoint.coords[i] - minPoint.coords[i]);
    }
    return a;
  }

  bool contains(const Point<DIM> &p) const {
    for (size_t i = 0; i < DIM; ++i) {
      if (p.coords[i] < minPoint.coords[i] ||
          p.coords[i] > maxPoint.coords[i]) {
        return false;
      }
    }
    return true;
  }

  bool intersects(const Rectangle<DIM> &other) const {
    for (size_t i = 0; i < DIM; ++i) {
      if (minPoint.coords[i] > other.maxPoint.coords[i] ||
          maxPoint.coords[i] < other.minPoint.coords[i]) {
        return false;
      }
    }
    return true;
  }

  // Returns a new rectangle that is the MBR of this and other
  Rectangle<DIM> enlarge(const Rectangle<DIM> &other) const {
    Point<DIM> newMin, newMax;
    for (size_t i = 0; i < DIM; ++i) {
      newMin.coords[i] = std::min(minPoint.coords[i], other.minPoint.coords[i]);
      newMax.coords[i] = std::max(maxPoint.coords[i], other.maxPoint.coords[i]);
    }
    return Rectangle<DIM>(newMin, newMax);
  }

  // Calculate area expansion needed to include other
  double expansionNeeded(const Rectangle<DIM> &other) const {
    Rectangle<DIM> enlarged = enlarge(other);
    return enlarged.area() - area();
  }

  // Distance from a point to the rectangle (0 if inside)
  double distanceSquared(const Point<DIM> &p) const {
    double dist = 0.0;
    for (size_t i = 0; i < DIM; ++i) {
      if (p.coords[i] < minPoint.coords[i]) {
        double d = minPoint.coords[i] - p.coords[i];
        dist += d * d;
      } else if (p.coords[i] > maxPoint.coords[i]) {
        double d = p.coords[i] - maxPoint.coords[i];
        dist += d * d;
      }
    }
    return dist;
  }
};

#endif // RECTANGLE_H
