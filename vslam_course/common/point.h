#pragma once

namespace zs {

template<class T>
class Point2D {
    public:
    Point2D() = default;
    Point2D(const T&x, const T& y):x(x), y(y) {}
    ~Point2D() {}
    T x, y;
};


template<class T>
class Point3D {
public:
    Point3D() = default;
    Point3D(const T&x, const T&y, const T& z):x(x), y(), z(z) {}

    T x, y, z;
};

using Point2D = Point2D<double>;
using Point3D = Point3D<double>;
using Point2Df = Point2D<double>;
using Point3Df = Point3D<double>;
};