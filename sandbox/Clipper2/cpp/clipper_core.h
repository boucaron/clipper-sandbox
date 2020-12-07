/*******************************************************************************
* Author    :  Angus Johnson                                                   *
* Version   :  10.0 (beta)                                                     *
* Date      :  21 November 2020                                                *
* Website   :  http://www.angusj.com                                           *
* Copyright :  Angus Johnson 2010-2020                                         *
* Purpose   :  Core Clipper Library structures and functions                   *
* License   :  http://www.boost.org/LICENSE_1_0.txt                            *
*                                                                              *
* C++       :  Thanks to help from Andreas Lücke - ALuecke@gmx.net             *
*******************************************************************************/

#ifndef CLIPPER_CORE_H
#define CLIPPER_CORE_H

#include "stdint.h"
#include <stdlib.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <iostream>
#include <limits>
#include <ostream>
#include <stdexcept>
#include <vector>
#include <type_traits>

namespace clipperlib {

const double floating_point_tolerance = 1E-15;           //floating point tolerance for equality
const double default_min_edge_len = 0.2;  //minimum edge length for stripping duplicates
const double sqrt_two = 1.4142135623731;
const double two_point_five_degrees = 0.0436332313;

// Point -----------------------------------------------------------------------

template <typename T>
struct Point;

using PointI = Point<int64_t>;
using PointD = Point<double>;

template <typename T>
struct Point {
    T x;
    T y;

    Point(T x = 0, T y = 0) :
        x(x), 
    y(y){};

    template <typename T2>
    Point<T>(Point<T2> p) : x(static_cast<T>(p.x)), y(static_cast<T>(p.y))
    {};

    Point &operator=(const Point &other) {
        x = other.x;
        y = other.y;
        return *this;
    }

    void Rotate(const PointD &center, double angle_rad);
    void Rotate(const PointD &center, double sin_a, double cos_a);

    friend inline bool operator==(const Point &a, const Point &b) {
        return a.x == b.x && a.y == b.y;
    }

    inline Point<T> operator-() const
    {
        return Point<T>(-x,-y);
    }

    inline PointD operator+(const PointD &b) const
    {
        return PointD(x+b.x, y+b.y);
    }
    inline PointD operator-(const PointD &b) const
    {
        return PointD(x-b.x, y-b.y);
    }

    inline PointD operator*(const double factor) const
    {
        return PointD(x*factor, y*factor);
    }

    friend inline bool operator!=(const Point &a, const Point &b) {
        return !(a == b);
    }
    friend inline bool operator<(const Point &a, const Point &b) {
        return (a.x == b.x) ? (a.y < b.y) : (a.x < b.x);
    }
    friend std::ostream &operator<<(std::ostream &os, const Point<T> &point) {
        os << "(" << point.x << "," << point.y << ")";
        return os;
    }
};

template <typename T>
PointI Round(Point<T> p)
{
    return PointI(static_cast<int64_t>(std::round(p.x)), static_cast<int64_t>(std::round(p.y)));
}

template <typename T>
inline bool NearEqual(const Point<T> p1, const Point<T> p2, double min_dist_sqrd) {
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) < min_dist_sqrd;
}


// Rect ------------------------------------------------------------------------

template <typename T>
struct Rect;

using RectI = Rect<int64_t>;
using RectD = Rect<double>;

template <typename T>
struct Rect {
    T left;
    T top;
    T right;
    T bottom;

    Rect() :
        left(0),
        top(0),
        right(0),
        bottom(0) {}

    Rect(T l, T t, T r, T b) :
        left(l),
        top(t),
        right(r),
        bottom(b) {}

    Rect &operator=(const Rect &other) {
        if (this != &other) {
            left = other.left;
            top = other.top;
            right = other.right;
            bottom = other.bottom;
        }
        return *this;
    }

    T Width() const { return right - left; }
    T Height() const { return bottom - top; }
    void Width(T _width) { right = left + _width; }
    void Height(T _height) { bottom = top + _height; }

    bool IsEmpty() const { return bottom <= top || right <= left; };

    void Inflate(T dx, T dy) {
        left -= dx;
        right += dx;
        top -= dy;
        bottom += dy;
    }

    void Offset(T dx, T dy) {
        left += dx;
        right += dx;
        top += dy;
        bottom += dy;
    }

    void Intersect(const Rect<T> &rect);
    void Union(const Rect<T> &rect);
    void Rotate(double angle_rad);
    void Scale(double scale) { left *= scale; top *= scale; right *= scale; bottom *= scale; };

    friend std::ostream &operator<<(std::ostream &os, const Rect<T> &rect) {
        os << "("
           << rect.left << "," << rect.top << "," << rect.right << "," << rect.bottom
           << ")";
        return os;
    }
};

// ClipperLibException ---------------------------------------------------------

class ClipperLibException : public std::exception {
public:
    ClipperLibException(const char *description) :
        m_descr(description) {}
    virtual const char *what() const throw() { return m_descr.c_str(); }

private:
    std::string m_descr;
};


// Path ------------------------------------------------------------------------

//Path: a simple data structure to represent a series of vertices, whether
//open (poly-line) or closed (polygon). A path may be simple or complex (self
//intersecting). For simple polygons, path orientation (whether clockwise or
//counter-clockwise) is generally used to differentiate outer paths from inner
//paths (holes). For complex polygons (and also for overlapping polygons),
//explicit 'filling rules' (see below) are used to indicate regions that are
//inside (filled) and regions that are outside (unfilled) a specific polygon.

template <typename T>
struct Path;
using PathI = Path<int64_t>;
using PathD = Path<double>;

template<typename T>
struct Path {
    std::vector<Point<T> > data;

    using Size = decltype(data.size());
    Size size() const { return data.size(); }
    void resize(Size size) { data.resize(size); }
    bool empty() const { return data.size() == 0; }
    void reserve(Size size) { data.reserve(size); }
    void push_back(const Point<T>& point) { data.push_back(point); }
    void pop_back() { data.pop_back(); }
    void clear() { data.clear(); }

    Path() {}

    Point<T> &operator[](Size idx) { return data[idx]; }
    const Point<T> &operator[](Size idx) const { return data[idx]; }

    void Append(const Path<T> &extra) {
      if (extra.size() > 0)
        data.insert(end(data), begin(extra.data), end(extra.data));
    }

    double Area() const;
    Rect<T> Bounds() const;
    void Offset(T dx, T dy);
    bool Orientation() const;
    void Reverse();
    void Rotate(const PointD &pivot, double angle_rad);
    void Scale(double sx, double sy);
    void StripDuplicates(bool is_closed_path = false, T min_length = 0);
    //void Trim(bool is_closed_path, T min_length);

    template<typename T2>        
    void AppendPointsScale(const Path<T2> & other, double scale)
    {
        data.reserve(data.size() + other.size());
        if  (std::numeric_limits<T>::is_integer)
        {
                for (const auto &p : other.data)
                    data.push_back(Point<T>(Round(p*scale)));
        }
        else
        {
                for (const auto &p : other.data)
                    data.push_back(Point<T>(p * scale));
        }
    }

    Path(const Path<T> & other, double scale){
        if (scale == 0) scale = 1;
        if (scale == 1) {
            Append(other);
        } else {
            AppendPointsScale(other,scale);
        }
    }

    template<typename T2, typename=
    typename std::enable_if<!std::is_same<T, T2>::value,T>::type >
    Path(const Path<T2> & other, double scale){
        if (scale == 0) scale = 1;
        AppendPointsScale(other,scale);
    }

    template<typename T2, typename=
    typename std::enable_if<!std::is_same<T, T2>::value,T>::type >
    void Assign(const Path<T2> & other, double scale){
        if (&other == reinterpret_cast<Path<T2>*>(this))
            throw ClipperLibException("Can't assign self to self in Path<T>::Assign.");
        data.clear();
        if (scale == 0) scale = 1;
        AppendPointsScale(other, scale);
    }

    void Assign(const Path<T> & other, double scale){
        if (&other == reinterpret_cast<Path<T>*>(this))
            throw ClipperLibException("Can't assign self to self in Path<T>::Assign.");
        data.clear();
        if (scale == 0) scale = 1;
        if (scale == 1) {
            Append(other);
        } else {
            AppendPointsScale(other, scale);
        }

    }

    friend inline Path<T> &operator<<(Path<T> &path, const Point<T> &point) {
        path.data.push_back(point);
        return path;
    }
    friend std::ostream &operator<<(std::ostream &os, const Path<T> &path) {
        if (path.data.empty())
            return os;

        Size last = path.size() - 1;

        for (Size i = 0; i < last; i++)
            os << "(" << path[i].x << "," << path[i].y << "), ";

        os << "(" << path[last].x << "," << path[last].y << ")\n";

        return os;
    }
};

// Paths -----------------------------------------------------------------------

template <typename T>
struct Paths;
using PathsI = Paths<int64_t>;
using PathsD = Paths<double>;

template <typename T>
struct Paths {
    std::vector<Path<T> > data;

    using Size = decltype(data.size());
    Size size() const { return data.size(); }
    void resize(Size size) { data.resize(size); }
    void reserve(Size size) { data.reserve(size); }
    void push_back(const Path<T> &path) { data.push_back(path); }
    void clear() { data.clear(); }

    Path<T> &operator[](Size idx) { return data[idx]; }
    const Path<T> &operator[](Size idx) const { return data[idx]; }

    Paths() {}
    Paths(const PathsI &other, double scale = 1.0);
    Paths(const PathsD &other, double scale = 1.0);

    void Append(const Paths<T> &extra);
    void Assign(const PathsI &other, double scale = 1.0);
    void Assign(const PathsD &other, double scale = 1.0);
    Rect<T> Bounds() const;
    void Offset(T dx, T dy);
    void Reverse();
    void Rotate(const PointD &center, double angle_rad);
    void Scale(double sx, double sy);
    void StripDuplicates(bool is_closed_path, T min_length);

    template<typename T2>
    void AppendPointsScale(const Paths<T2>& other, double scale) {
        size_t other_size = other.size();
        data.resize(other_size);
        for (size_t i = 0; i < other_size; ++i)            
            data[i].AppendPointsScale(other[i], scale);

    }

    friend inline Paths<T> &operator<<(Paths<T> &paths, const Path<T> &path) {
        paths.data.push_back(path);
        return paths;
    }

    friend std::ostream &operator<<(std::ostream &os, const Paths<T> &paths) {
        for (Size i = 0; i < paths.size(); i++)
            os << paths[i];
        os << "\n";
        return os;
    }
};

// PathsArray ------------------------------------------------------------------

template <typename T>
struct PathsArray {
    std::vector<Paths<T> > data;

    using Size = decltype(data.size());

    Size size() const { return data.size(); }
    void resize(Size size) { data.resize(size); }
    void reserve(Size size) { data.reserve(size); }
    void push_back(const Paths<T> &paths) { data.push_back(paths); }
    void clear() { data.clear(); }

    Paths<T> &operator[](Size idx) { return data[idx]; }
    const Paths<T> &operator[](Size idx) const { return data[idx]; }

    Rect<T> Bounds() const;
};

using PathsArrayI = PathsArray<int64_t>;
using PathsArrayD = PathsArray<double>;


// Miscellaneous ---------------------------------------------------------------

template <typename T>
T CrossProduct(const Point<T>& pt1, const Point<T>& pt2, const Point<T>& pt3) {
    return ((pt2.x - pt1.x) * (pt3.y - pt2.y) - (pt2.y - pt1.y) * (pt3.x - pt2.x));
}
//------------------------------------------------------------------------------

template <typename T>
double DistanceSqr(const Point<T> pt1, const Point<T> pt2) {
    return std::pow(pt1.x - pt2.x, 2.0) + std::pow(pt1.y - pt2.y, 2.0);
}
//------------------------------------------------------------------------------

template <typename T>
double DistanceFromLineSqrd(const Point<T> &pt, const Point<T> &ln1, const Point<T> &ln2)
{
    //perpendicular distance of point (x³,y³) = (Ax³ + By³ + C)/Sqrt(A² + B²)
    //see http://en.wikipedia.org/wiki/Perpendicular_distance
    double A = (ln1.y - ln2.y);
    double B = (ln2.x - ln1.x);
    double C = A * ln1.x + B * ln1.y;
    C = A * pt.x + B * pt.y - C;
    return (C * C) / (A * A + B * B);
}
//---------------------------------------------------------------------------

template <typename T>
bool NearCollinear(const Point<T> &pt1, const Point<T> &pt2, const Point<T> &pt3, double sin_sqrd_min_angle_rads)
{
    double cp = std::abs(CrossProduct(pt1, pt2, pt3));
    return (cp * cp) / (DistanceSqr(pt1, pt2) * DistanceSqr(pt2, pt3)) < sin_sqrd_min_angle_rads;
}
//------------------------------------------------------------------------------

template <typename T>
Path<T> CleanPath(const Path<T>& path, bool is_closed,
    double min_length = sqrt_two, double min_angle = two_point_five_degrees)
{
    Path<T> result;
    size_t len = path.size();
    if (len < 2) return result;
    double distSqrd = min_length * min_length;
    double sin_sqrd_min_angle = sin(min_angle);
    sin_sqrd_min_angle *= sin_sqrd_min_angle;

    result.reserve(len);
    result.push_back(path[0]);
    result.push_back(path[1]);
    len = 2;

    //clean up colinear edges
    typename std::vector<Point<T>>::const_iterator cit;
    for (cit = path.data.cbegin() + 2; cit != path.data.cend(); ++cit)
    {
        if (NearCollinear(result[len - 2], result[len - 1], *cit, sin_sqrd_min_angle))
        {
            result[len - 1] = *cit;
        }
        else
        {
            result.push_back(*cit);
            ++len;
        }
    }

    //clean up insignificant edges
    typename std::vector<Point<T>>::const_iterator it;
    for (it = result.data.begin() + 1; it != result.data.end(); )
    {
        if (NearEqual(*(it - 1), *(it), distSqrd))
        {
            it = result.data.erase(it);
            --len;
        }
        else
            ++it;
    }
    if (is_closed && NearEqual(result[0], result[len - 1], distSqrd)) result.pop_back();

    return result;
}
//------------------------------------------------------------------------------

//Note: all clipping operations except for Difference are commutative.
enum class ClipType { None, Intersection, Union, Difference, Xor };

enum class PathType { Subject, Clip };

//By far the most widely used filling rules for polygons are EvenOdd
//and NonZero, sometimes called Alternate and Winding respectively.
//https://en.wikipedia.org/wiki/Nonzero-rule
enum class FillRule { EvenOdd, NonZero, Positive, Negative };

//PointInPolygon
enum class PipResult { Inside, Outside, OnEdge };

PipResult PointInPolygon(const PointI &pt, const PathI &path);


}  // namespace clipperlib

#endif  // CLIPPER_CORE_H
