#ifndef POINT_H
#define POINT_H

#include <vector>
#include <cmath>
#include <iostream>
#include <initializer_list>

template <size_t DIM>
class Point {
public:
    double coords[DIM];

    Point() {
        for (size_t i = 0; i < DIM; ++i) coords[i] = 0.0;
    }

    Point(std::initializer_list<double> list) {
        size_t i = 0;
        for (auto val : list) {
            if (i < DIM) coords[i++] = val;
        }
    }

    double get(size_t index) const {
        return coords[index];
    }

    // Euclidean distance squared (to avoid sqrt when comparing)
    double distanceSquared(const Point<DIM>& other) const {
        double dist = 0.0;
        for (size_t i = 0; i < DIM; ++i) {
            double d = coords[i] - other.coords[i];
            dist += d * d;
        }
        return dist;
    }

    double distance(const Point<DIM>& other) const {
        return std::sqrt(distanceSquared(other));
    }

    bool operator==(const Point<DIM>& other) const {
        for (size_t i = 0; i < DIM; ++i) {
            if (std::abs(coords[i] - other.coords[i]) > 1e-9) return false;
        }
        return true;
    }
};

#endif // POINT_H
