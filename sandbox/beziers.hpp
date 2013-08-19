/*******************************************************************************
*                                                                              *
* Author    :  Angus Johnson                                                   *
* Version   :  0.9 (alpha)                                                     *
* Date      :  10 August 2013                                                  *
* Website   :  http://www.angusj.com                                           *
* Copyright :  Angus Johnson 2010-2013                                         *
*                                                                              *
* License:                                                                     *
* Use, modification & distribution is subject to Boost Software License Ver 1. *
* http://www.boost.org/LICENSE_1_0.txt                                         *
*                                                                              *
*******************************************************************************/

#ifndef beziers_hpp
#define beziers_hpp

#include <vector>
#include "clipper.hpp"

namespace BezierLib {

  using namespace ClipperLib;

  enum BezierType {CubicBezier, QuadBezier};
  const double DefaultPrecision = 0.5;

  class Bezier;

  class BezierList
  {
  private:
    std::vector <Bezier*> m_Beziers;
    double m_Precision;
  public:
    BezierList(double precision = DefaultPrecision);
    ~BezierList();
    int AddPath(const Path ctrlPts, BezierType bezType);
    void Clear();

    void GetCtrlPts(int index, Path& path);
    BezierType GetBezierType(int index);
    void GetFlattenedPath(int index, Path& path);
    static void Flatten(const Path& in_path, Path& out_path, 
      BezierType bezType, double precision = DefaultPrecision);
    static void Flatten(const Paths& in_paths, Paths& out_paths, 
      BezierType bezType, double precision = DefaultPrecision);
    void Reconstruct(cInt z1, cInt z2, Path& path);
    double Precision();
    void Precision(double value);
  };

} //BezierLib namespace
#endif //bezier_hpp
