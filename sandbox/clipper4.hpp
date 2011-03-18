/*******************************************************************************
*                                                                              *
* Author    :  Angus Johnson                                                   *
* Version   :  4.0.6 (beta)                                                    *
* Date      :  18 March 2011                                                   *
* Website   :  http://www.angusj.com                                           *
* Copyright :  Angus Johnson 2010-2011                                         *
*                                                                              *
* License:                                                                     *
* Use, modification & distribution is subject to Boost Software License Ver 1. *
* http://www.boost.org/LICENSE_1_0.txt                                         *
*                                                                              *
* Attributions:                                                                *
* The code in this library is an extension of Bala Vatti's clipping algorithm: *
* "A generic solution to polygon clipping"                                     *
* Communications of the ACM, Vol 35, Issue 7 (July 1992) pp 56-63.             *
* http://portal.acm.org/citation.cfm?id=129906                                 *
*                                                                              *
* Computer graphics and geometric modeling: implementation and algorithms      *
* By Max K. Agoston                                                            *
* Springer; 1 edition (January 4, 2005)                                        *
* http://books.google.com/books?q=vatti+clipping+agoston                       *
*                                                                              *
*******************************************************************************/

#ifndef clipper4_hpp
#define clipper4_hpp

#include <vector>
#include <stdexcept>
#include <cstring>
#include <cstdlib>

namespace clipper4 {

enum ClipType { ctIntersection, ctUnion, ctDifference, ctXor };
enum PolyType { ptSubject, ptClip };
enum PolyFillType { pftEvenOdd, pftNonZero };


struct IntPoint {int X; int Y;};
typedef std::vector< IntPoint > Polygon;
typedef std::vector< Polygon > Polygons;

typedef signed long long long64;

bool IsClockwise(const Polygon &poly);
double Area(const Polygon &poly);

//used internally ...
enum EdgeSide { esLeft, esRight };
enum IntersectProtects { ipNone = 0, ipLeft = 1, ipRight = 2, ipBoth = 3 };

struct TEdge4 {
  int xbot;
  int ybot;
  int xcurr;
  int ycurr;
  int xtop;
  int ytop;
  double dx;
  int tmpX;
  PolyType polyType;
  EdgeSide side;
  int windDelta; //1 or -1 depending on winding direction
  int windCnt;
  int windCnt2; //winding count of the opposite polytype
  int outIdx;
  TEdge4 *next;
  TEdge4 *prev;
  TEdge4 *nextInLML;
  TEdge4 *nextInAEL;
  TEdge4 *prevInAEL;
  TEdge4 *nextInSEL;
  TEdge4 *prevInSEL;
};

struct IntersectNode {
  TEdge4          *edge1;
  TEdge4          *edge2;
  IntPoint        pt;
  IntersectNode  *next;
};

struct LocalMinima {
  int           Y;
  TEdge4        *leftBound;
  TEdge4        *rightBound;
  LocalMinima  *next;
};

struct Scanbeam {
  int       Y;
  Scanbeam *next;
};

struct PolyPt {
  IntPoint pt;
  PolyPt  *next;
  PolyPt  *prev;
  bool     isDone;
};

struct Tracer {
  bool    dirF; //direction (forward = true)
  PolyPt *pp;
  Tracer *next;
  Tracer *prev;
};

struct HoleInfo {
  PolyPt *pp;
  bool    isHole;
  int     idx;
  bool    isDone;
};

typedef std::vector < PolyPt* > PolyPtList;
typedef std::vector < HoleInfo* > HoleInfoList;
typedef std::vector < TEdge4* > EdgeList;

//ClipperBase is the ancestor to the Clipper class. It should not be
//instantiated directly. This class simply abstracts the conversion of sets of
//polygon coordinates into edge objects that are stored in a LocalMinima list.
class Clipper4Base
{
public:
  Clipper4Base();
  virtual ~Clipper4Base();
  void AddPolygon(const Polygon &pg, PolyType polyType);
  void AddPolygons( const Polygons &ppg, PolyType polyType);
  virtual void Clear();
protected:
  void DisposeLocalMinimaList();
  TEdge4* AddBoundsToLML(TEdge4 *e);
  void PopLocalMinima();
  virtual bool Reset();
  void InsertLocalMinima(LocalMinima *newLm);
  LocalMinima           *m_CurrentLM;
  LocalMinima           *m_MinimaList;
private:
  EdgeList               m_edges;
};

class Clipper4 : public virtual Clipper4Base
{
public:
  Clipper4();
  virtual ~Clipper4();
  bool Execute(ClipType clipType,
    Polygons &solution,
    PolyFillType subjFillType = pftEvenOdd,
    PolyFillType clipFillType = pftEvenOdd);
protected:
  bool Reset();
private:
  PolyPtList        m_PolyPts;
  HoleInfoList      m_HoleInfos;
  ClipType          m_ClipType;
  Scanbeam         *m_Scanbeam;
  TEdge4            *m_ActiveEdges;
  TEdge4            *m_SortedEdges;
  IntersectNode    *m_IntersectNodes;
  bool              m_ExecuteLocked;
  PolyFillType      m_ClipFillType;
  PolyFillType      m_SubjFillType;
  Tracer           *m_Tracers;
  Tracer           *m_TracersEnd;
  void DisposeScanbeamList();
  void SetWindingCount(TEdge4& edge);
  bool IsNonZeroFillType(const TEdge4& edge) const;
  bool IsNonZeroAltFillType(const TEdge4& edge) const;
  void InsertScanbeam(const int Y);
  int PopScanbeam();
  void InsertLocalMinimaIntoAEL(const int botY);
  void InsertEdgeIntoAEL(TEdge4 *edge);
  void AddEdgeToSEL(TEdge4 *edge);
  void CopyAELToSEL();
  void DeleteFromSEL(TEdge4 *e);
  void DeleteFromAEL(TEdge4 *e);
  void UpdateEdgeIntoAEL(TEdge4 *&e);
  void SwapPositionsInSEL(TEdge4 *edge1, TEdge4 *edge2);
  bool IsContributing(const TEdge4& edge) const;
  bool IsTopHorz(const int XPos);
  void SwapPositionsInAEL(TEdge4 *edge1, TEdge4 *edge2);
  void DoMaxima(TEdge4 *e, int topY);
  void ProcessHorizontals();
  void ProcessHorizontal(TEdge4 *horzEdge);
  void AddLocalMaxPoly(TEdge4 *e1, TEdge4 *e2, const IntPoint &pt);
  void AddLocalMinPoly(TEdge4 *e1, TEdge4 *e2, const IntPoint &pt);
  void AppendPolygon(TEdge4 *e1, TEdge4 *e2);
  void DoEdge1(TEdge4 *edge1, TEdge4 *edge2, const IntPoint &pt);
  void DoEdge2(TEdge4 *edge1, TEdge4 *edge2, const IntPoint &pt);
  void DoBothEdges(TEdge4 *edge1, TEdge4 *edge2, const IntPoint &pt);
  void IntersectEdges(TEdge4 *e1, TEdge4 *e2,
    const IntPoint &pt, IntersectProtects protects);
  PolyPt* AddPolyPt(TEdge4 *e, const IntPoint &pt);
  void DisposeAllPolyPts();
  bool ProcessIntersections( const int topY);
  void AddIntersectNode(TEdge4 *e1, TEdge4 *e2, const IntPoint &pt);
  void BuildIntersectList(const int topY);
  void ProcessIntersectList();
  void ProcessEdgesAtTopOfScanbeam(const int topY);
  void BuildResult(Polygons& polypoly);
  void DisposeIntersectNodes();
  bool FixupIntersections();
  void AddTracer(PolyPt &p, bool isForward);
  void DeleteTracer(Tracer *t);
  void UpdateTracer(Tracer *t, int currY);
  void GetHoleStates();
};

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

class clipper4Exception : public std::exception
{
  public:
    clipper4Exception(const char* description)
      throw(): std::exception(), m_description (description) {}
    virtual ~clipper4Exception() throw() {}
    virtual const char* what() const throw() {return m_description.c_str();}
  private:
    std::string m_description;
};
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

} //clipper4 namespace
#endif //clipper4_hpp


