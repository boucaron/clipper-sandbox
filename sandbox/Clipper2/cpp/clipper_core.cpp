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

#include "clipper_core.h"

namespace clipperlib {

//------------------------------------------------------------------------------
// Template specialization declarations ...
//------------------------------------------------------------------------------

template struct Point<int64_t>;
template struct Point<double>;
template struct Rect<int64_t>;
template struct Rect<double>;
template struct Path<int64_t>;
template struct Path<double>;
template struct Paths<int64_t>;
template struct Paths<double>;
template struct PathsArray<int64_t>;
template struct PathsArray<double>;

//------------------------------------------------------------------------------
// Point
//------------------------------------------------------------------------------

template<typename T>
inline void Point<T>::Rotate(const PointD & center, double angle_rad){
    double tmp_x = x - center.x;
    double tmp_y = y - center.y;
    double cos_a = cos(angle_rad);
    double sin_a = sin(angle_rad);

    if  (std::numeric_limits<T>::is_integer) {
        x = static_cast<T>(std::round(tmp_x * cos_a - tmp_y * sin_a + center.x));
        y = static_cast<T>(std::round(tmp_x * sin_a - tmp_y * cos_a + center.y));
    }
    else {
        x = static_cast<T>(tmp_x * cos_a - tmp_y * sin_a + center.x);
        y = static_cast<T>(tmp_x * sin_a - tmp_y * cos_a + center.y);
    }
}

template<typename T>
inline void Point<T>::Rotate(const PointD & center,
          double sin_a, double cos_a){
    double tmp_x = x - center.x;
    double tmp_y = y - center.y;

    if  (std::numeric_limits<T>::is_integer) {
        x = static_cast<T>(std::round(tmp_x * cos_a - tmp_y * sin_a + center.x));
        y = static_cast<T>(std::round(tmp_x * sin_a - tmp_y * cos_a + center.y));
    }
    else {
        x = static_cast<T>(tmp_x * cos_a - tmp_y * sin_a + center.x);
        y = static_cast<T>(tmp_x * sin_a - tmp_y * cos_a + center.y);
    }
}

//------------------------------------------------------------------------------
// Rect
//------------------------------------------------------------------------------

template <typename T>
void Rect<T>::Intersect(const Rect<T> &rect) {
    if (IsEmpty())
        return;
    else if (rect.IsEmpty()) {
        *this = Rect();
    } else {
        left = std::max(rect.left, left);
        right = std::min(rect.right, right);
        top = std::max(rect.top, top);
        bottom = std::min(rect.bottom, bottom);
        if (IsEmpty())
            *this = Rect();
    }
}
//------------------------------------------------------------------------------
template<typename T>
inline void Rect<T>::Rotate(double angle_rad) {
    using UsedT = typename std::conditional<std::numeric_limits<T>::is_integer, double, T>::type;
    Point<UsedT> cp;
    cp.x = static_cast<UsedT>((right + left) / 2);
    cp.y = static_cast<UsedT>((bottom + top) / 2);

    Path<UsedT> pts;
    pts.resize(4);
    pts[0] = Point<UsedT>(static_cast<UsedT>(left), static_cast<UsedT>(top));
    pts[1] = Point<UsedT>(static_cast<UsedT>(right), static_cast<UsedT>(top));
    pts[2] = Point<UsedT>(static_cast<UsedT>(right), static_cast<UsedT>(bottom));
    pts[3] = Point<UsedT>(static_cast<UsedT>(left), static_cast<UsedT>(bottom));

    pts.Rotate(cp, angle_rad);

    const auto resultx = std::minmax_element(begin(pts.data), end(pts.data),[](Point<UsedT> p1, Point<UsedT> p2) {return p1.x< p2.x;});
    const auto resulty = std::minmax_element(begin(pts.data), end(pts.data),[](Point<UsedT> p1, Point<UsedT> p2) {return p1.y< p2.y;});

    if  (std::numeric_limits<T>::is_integer) {
        left = static_cast<T>(std::floor(resultx.first->x));
        right = static_cast<T>(std::ceil(resultx.second->x));
        top = static_cast<T>(std::floor(resulty.first->y));
        bottom = static_cast<T>(std::ceil(resulty.second->y));
    }
    else
    {
        left = static_cast<T>(resultx.first->x);
        right = static_cast<T>(resultx.second->x);
        top = static_cast<T>(resulty.first->y);
        bottom = static_cast<T>(resulty.second->y);
    }
}


//------------------------------------------------------------------------------

template <typename T>
void Rect<T>::Union(const Rect<T> &rect) {
    if (rect.IsEmpty())
        return;
    else if (IsEmpty()) {
        *this = rect;
        return;
    }
    left = std::min(rect.left, left);
    right = std::max(rect.right, right);
    top = std::min(rect.top, top);
    bottom = std::max(rect.bottom, bottom);
}

//------------------------------------------------------------------------------
// Path
//------------------------------------------------------------------------------

template <typename T>
double Path<T>::Area() const {
    double area = 0.0;
    auto len = data.size() - 1;
    if (len < 2) return area;
    auto j = len;
    for (decltype(len) i = 0; i <= len; ++i) {
        double d = (double)(data[j].x + data[i].x);
        area += d * (data[j].y - data[i].y);
        j = i;
    }
    return -area * 0.5;
}
//------------------------------------------------------------------------------

template <typename T>
Rect<T> Path<T>::Bounds() const {
    const T _MAX = std::numeric_limits<T>::max();
    const T _MIN = -_MAX;

    Rect<T> bounds(_MAX, _MAX, _MIN, _MIN);

    for (const auto &point : data) {
        if (point.x < bounds.left) bounds.left = point.x;
        if (point.x > bounds.right) bounds.right = point.x;
        if (point.y < bounds.top) bounds.top = point.y;
        if (point.y > bounds.bottom) bounds.bottom = point.y;
    }

    if (bounds.left >= bounds.right)
        return Rect<T>();
    else
        return bounds;
}
//------------------------------------------------------------------------------

template <typename T>
void Path<T>::Offset(T dx, T dy) {
    if (dx == 0 && dy == 0) return;
    for (auto &point : data) {
        point.x += dx;
        point.y += dy;
    }
}
//------------------------------------------------------------------------------

template <typename T>
bool Path<T>::Orientation() const {
    return Area() >= 0;
}
//------------------------------------------------------------------------------

template <typename T>
void Path<T>::Reverse() {
    std::reverse(begin(data), end(data));
}
//------------------------------------------------------------------------------

template <typename T>
void Path<T>::Rotate(const PointD &center, double angle_rad) {
    double cos_a = cos(angle_rad);
    double sin_a = sin(angle_rad);

    for (auto &point : data)
        point.Rotate(center, sin_a, cos_a);
}
//------------------------------------------------------------------------------

template <typename T>
void Path<T>::Scale(double sx, double sy) {
    if (sx == 0) sx = 1;
    if (sy == 0) sy = 1;
    if (sx == 1 && sy == 1) return;

    if  (std::numeric_limits<T>::is_integer)
    {
        for (auto& point : data) {
            point.x = static_cast<T>(std::round(point.x * sx));
            point.y = static_cast<T>(std::round(point.y * sy));
        }
    }
    else
    {
        for (auto& point : data) {
            point.x = static_cast<T>(point.x * sx);
            point.y = static_cast<T>(point.y * sy);
        }
    }
    StripDuplicates();
}
//------------------------------------------------------------------------------

template <typename T>
void Path<T>::StripDuplicates(bool is_closed_path, T min_length) {
    if (std::numeric_limits<T>::is_integer && min_length < 1)
    {
        data.erase(unique(begin(data), end(data)), end(data));
    }
    else
    {
        if (data.size() < 2) return;
        if (min_length < floating_point_tolerance)
            min_length = default_min_edge_len;
        for (auto it = data.begin() + 1; it != data.end(); )
            if (NearEqual(*(it - 1), *it, min_length * min_length))
                it = data.erase(it);
            else
                ++it;
    }
    size_t len = data.size();
    if (!is_closed_path || len == 0) return;
    if (NearEqual(data[0], data[len - 1], min_length * min_length))
        data.resize(len - 1);
}

//------------------------------------------------------------------------------
// Paths
//------------------------------------------------------------------------------

template<>
void PathsI::Assign(const PathsI &other, double scale) {
    using namespace std;
    data.clear();
    data.resize(other.data.size());
    typename vector<PathI>::iterator it1;
    typename vector<PathI>::const_iterator it2;
    for (it1 = data.begin(), it2 = other.data.begin(); it1 != data.end(); it1++, it2++)
        it1->Assign(*it2, scale);
}
//------------------------------------------------------------------------------

template<>
void PathsD::Assign(const PathsI &other, double scale) {
    using namespace std;
    data.clear();
    data.resize(other.data.size());
    typename vector<PathD>::iterator it1;
    typename vector<PathI>::const_iterator it2;
    for (it1 = data.begin(), it2 = other.data.begin(); it1 != data.end(); it1++, it2++)
        it1->Assign(*it2, scale);
}
//------------------------------------------------------------------------------
template<>
void PathsI::Assign(const PathsD &other, double scale) {
    using namespace std;
    data.clear();
    data.resize(other.data.size());
    typename vector<PathI>::iterator it1;
    typename vector<PathD>::const_iterator it2;
    for (it1 = data.begin(), it2 = other.data.begin(); it1 != data.end(); it1++, it2++)
        it1->Assign(*it2, scale);
}
//------------------------------------------------------------------------------

template <>
void PathsD::Assign(const PathsD &other, double scale) {
    using namespace std;
    data.clear();
    data.resize(other.data.size());
    typename vector<PathD>::iterator it1;
    typename vector<PathD>::const_iterator it2;
    for (it1 = data.begin(), it2 = other.data.begin(); it1 != data.end(); it1++, it2++)
        it1->Assign(*it2, scale);
}
//------------------------------------------------------------------------------

template <>
PathsI::Paths(const PathsI &other, double scale) {
  Assign(other, scale);
}
//------------------------------------------------------------------------------

template <>
PathsD::Paths(const PathsI &other, double scale) {
  Assign(other, scale);
}
//------------------------------------------------------------------------------

template <>
PathsI::Paths(const PathsD &other, double scale) {
  Assign(other, scale);
}
//------------------------------------------------------------------------------

template <>
PathsD::Paths(const PathsD &other, double scale) {
  Assign(other, scale);
}
//------------------------------------------------------------------------------

template<typename T>
clipperlib::Paths<T>::Paths(const PathsI & other, double scale){}
//------------------------------------------------------------------------------

template<typename T>
clipperlib::Paths<T>::Paths(const PathsD & other, double scale){}
//------------------------------------------------------------------------------

template <typename T>
void Paths<T>::Append(const Paths<T> &extra) {
  if (extra.size() > 0)
    data.insert(end(data), begin(extra.data), end(extra.data));
}
//------------------------------------------------------------------------------

template<typename T>
void clipperlib::Paths<T>::Assign(const PathsI & other, double scale){}
//------------------------------------------------------------------------------

template<typename T>
void clipperlib::Paths<T>::Assign(const PathsD & other, double scale){}
//------------------------------------------------------------------------------

template <typename T>
Rect<T> Paths<T>::Bounds() const {
    const T _MAX = std::numeric_limits<T>::max();
    const T _MIN = -_MAX;

    Rect<T> bounds(_MAX, _MAX, _MIN, _MIN);

    for (const auto &path : data) {
        for (const auto &point : path.data) {
            if (point.x < bounds.left) bounds.left = point.x;
            if (point.x > bounds.right) bounds.right = point.x;
            if (point.y < bounds.top) bounds.top = point.y;
            if (point.y > bounds.bottom) bounds.bottom = point.y;
        }
    }

    if (bounds.left >= bounds.right)
        return Rect<T>();
    else
        return bounds;
}
//------------------------------------------------------------------------------

template <typename T>
void Paths<T>::Rotate(const PointD &center, double angle_rad) {
    double cos_a = cos(angle_rad);
    double sin_a = sin(angle_rad);

    for (auto &path : data)
        for (auto &point : path.data)
            point.Rotate(center, sin_a, cos_a);
}
//------------------------------------------------------------------------------

template <typename T>
void Paths<T>::Scale(double scale_x, double scale_y) {
    for (auto &path : data)
        path.Scale(scale_x, scale_y);
}
//------------------------------------------------------------------------------

template <typename T>
void Paths<T>::Offset(T dx, T dy) {
    if (dx == 0 && dy == 0) return;
    for (auto &path : data)
        for (auto &point : path.data) {
            point.x += dx;
            point.y += dy;
        }
}
//------------------------------------------------------------------------------

template <typename T>
void Paths<T>::Reverse() {
    for (auto &path : data)
        path.Reverse();
}
//------------------------------------------------------------------------------

template <typename T>
void Paths<T>::StripDuplicates(bool is_closed_path, T min_length) {
    for (auto& path : data)
        path.StripDuplicates(is_closed_path, min_length);
}

//------------------------------------------------------------------------------
// PathsArray
//------------------------------------------------------------------------------

template <typename T>
Rect<T> PathsArray<T>::Bounds() const {
    const T _MAX = std::numeric_limits<T>::max();
    const T _MIN = -_MAX;

    Rect<T> bounds(_MAX, _MAX, _MIN, _MIN);

    for (const auto &paths : data) {
        for (const auto &path : paths.data) {
            for (const auto &point : path.data) {
                if (point.x < bounds.left) bounds.left = point.x;
                if (point.x > bounds.right) bounds.right = point.x;
                if (point.y < bounds.top) bounds.top = point.y;
                if (point.y > bounds.bottom) bounds.bottom = point.y;
            }
        }
    }

    if (bounds.left >= bounds.right)
        return Rect<T>();
    else
        return bounds;
}

//------------------------------------------------------------------------------
// Miscellaneous
//------------------------------------------------------------------------------

PipResult PointInPolygon(const PointI &pt, const PathI &path) {
    int val = 0;
    auto cnt = path.size();
    double d, d2, d3;  // using doubles to avoid possible integer overflow
    PointI ip, ip_next;
    PipResult result = PipResult::OnEdge;

    if (cnt < 3) {
        result = PipResult::Outside;
        return result;
    }
    ip = path[0];
    for (decltype(cnt) i = 1; i < cnt; ++i) {
        if (i < cnt)
            ip_next = path[i];
        else
            ip_next = path[0];

        if (ip_next.y == pt.y) {
            if ((ip_next.x == pt.x) || ((ip.y == pt.y) && ((ip_next.x > pt.x) == (ip.x < pt.x)))) {
                return result;
            }
        }
        if ((ip.y < pt.y) != (ip_next.y < pt.y)) {
            if (ip.x >= pt.x) {
                if (ip_next.x > pt.x) {
                    val = 1 - val;
                } else {
                    d2 = (double)(ip.x - pt.x);
                    d3 = (double)(ip_next.x - pt.x);
                    d = d2 * (double)(ip_next.y - pt.y) - d3 * (double)(ip.y - pt.y);
                    if (d == 0)
                        return result;
                    if ((d > 0) == (ip_next.y > ip.y))
                        val = 1 - val;
                }
            } else {
                if (ip_next.x > pt.x) {
                    d2 = (double)(ip.x - pt.x);
                    d3 = (double)(ip_next.x - pt.x);
                    d = d2 * (double)(ip_next.y - pt.y) - d3 * (double)(ip.y - pt.y);
                    if (d == 0)
                        return result;
                    if ((d > 0) == (ip_next.y > ip.y))
                        val = 1 - val;
                }
            }
        }
        ip = ip_next;
    }
    switch (val) {
    case -1: result = PipResult::OnEdge; break;
        case 1: result = PipResult::Inside; break;
        default: result = PipResult::Outside; break;
    }
    return result;
}
//------------------------------------------------------------------------------

}  // namespace clipperlib
