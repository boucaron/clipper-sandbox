/*******************************************************************************
*                                                                              *
* Author    :  Angus Johnson                                                   *
* Version   :  4.0.7 (beta)                                                    *
* Date      :  19 March 2011                                                   *
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

/*******************************************************************************
*                                                                              *
* This is a translation of the Delphi Clipper library and the naming style     *
* used has retained a Delphi flavour.                                          *
*                                                                              *
*******************************************************************************/

#include "clipper4.hpp"
#include <cmath>
#include <ctime>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <cstring>
#include <cstdlib>

namespace clipper4 {

static double const horizontal = -3.4E+38;
enum Direction { dRightToLeft, dLeftToRight };

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

bool IsClockwise(const Polygon &poly)
{
  int highI = poly.size() -1;
  if (highI < 2) return false;
  double area = (double)poly[highI].X * poly[0].Y -(double)poly[0].X * poly[highI].Y;
  for (int i = 0; i < highI; ++i)
    area += (double)poly[i].X * poly[i+1].Y -(double)poly[i+1].X * poly[i].Y;
  //area := area/2;
  return area > 0; //ie reverse of normal formula because Y axis inverted
}
//------------------------------------------------------------------------------

double Area(const Polygon &poly)
{
  int highI = poly.size() -1;
  if (highI < 2) return false;
  double area =
    (long64)poly[highI].X * poly[0].Y - (long64)poly[0].X * poly[highI].Y;
  for (int i = 0; i < highI; ++i)
    area += (long64)poly[i].X * poly[i+1].Y - (long64)poly[i+1].X * poly[i].Y;
  return area/2;
}
//------------------------------------------------------------------------------

IntPoint MakePoint(const int X, const int Y)
{
  IntPoint p;
  p.X = X;
  p.Y = Y;
  return p;
}
//------------------------------------------------------------------------------

bool SlopesEqual(TEdge4 &e1, TEdge4 &e2)
{
  if (e1.ybot == e1.ytop) return (e2.ybot == e2.ytop);
  else if (e2.ybot == e2.ytop) return false;
  else return (long64)(e1.ytop - e1.ybot)*(e2.xtop - e2.xbot) -
      (long64)(e1.xtop - e1.xbot)*(e2.ytop - e2.ybot) == 0;
}
//------------------------------------------------------------------------------

bool SlopesEqual(const IntPoint pt1, const IntPoint pt2, const IntPoint pt3)
{
  if (pt1.Y == pt2.Y) return (pt2.Y == pt3.Y);
  else if (pt2.Y == pt3.Y) return false;
  else return
    long64(pt1.Y-pt2.Y)*(pt2.X-pt3.X) - long64(pt1.X-pt2.X)*(pt2.Y-pt3.Y) == 0;
}
//------------------------------------------------------------------------------

void SetDx(TEdge4 &e)
{
  if (e.ybot == e.ytop) e.dx = horizontal;
  else e.dx = (double)(e.xtop - e.xbot)/(e.ytop - e.ybot);
}
//---------------------------------------------------------------------------

double GetDx(const IntPoint pt1, const IntPoint pt2)
{
  if (pt1.Y == pt2.Y) return horizontal;
  else return (double)(pt2.X - pt1.X)/(pt2.Y - pt1.Y);
}
//---------------------------------------------------------------------------

void SwapSides(TEdge4 &edge1, TEdge4 &edge2)
{
  EdgeSide side =  edge1.side;
  edge1.side = edge2.side;
  edge2.side = side;
}
//------------------------------------------------------------------------------

void SwapPolyIndexes(TEdge4 &edge1, TEdge4 &edge2)
{
  int outIdx =  edge1.outIdx;
  edge1.outIdx = edge2.outIdx;
  edge2.outIdx = outIdx;
}
//------------------------------------------------------------------------------

inline int Round(double val)
{
  if ((val < 0)) return (int)(val - 0.5); else return (int)(val + 0.5);
}
//------------------------------------------------------------------------------

int TopX(TEdge4 &edge, const int currentY)
{
  if( currentY == edge.ytop ) return edge.xtop;
  return edge.xbot + Round(edge.dx *(currentY - edge.ybot));
}
//------------------------------------------------------------------------------

int TopX(const IntPoint pt1, const IntPoint pt2, const int currentY)
{
  //preconditions: pt1.Y <> pt2.Y and pt1.Y > pt2.Y
  if (currentY >= pt1.Y) return pt1.X;
  else if (currentY == pt2.Y) return pt2.X;
  else if (pt1.X == pt2.X) return pt1.X;
  else
  {
    double q = (pt1.X-pt2.X)/(pt1.Y-pt2.Y);
    return (int)(pt1.X + (currentY - pt1.Y) *q);
  }
}
//------------------------------------------------------------------------------

bool IntersectPoint(TEdge4 &edge1, TEdge4 &edge2, IntPoint &ip)
{
  double b1, b2;
  if (SlopesEqual(edge1, edge2)) return false;
  else if (edge1.dx == 0)
  {
    ip.X = edge1.xbot;
    if (edge2.dx == horizontal)
    {
      ip.Y = edge2.ybot;
    } else
    {
      b2 = edge2.ybot - (edge2.xbot/edge2.dx);
      ip.Y = Round(ip.X/edge2.dx + b2);
    }
  }
  else if (edge2.dx == 0)
  {
    ip.X = edge2.xbot;
    if (edge1.dx == horizontal)
    {
      ip.Y = edge1.ybot;
    } else
    {
      b1 = edge1.ybot - (edge1.xbot/edge1.dx);
      ip.Y = Round(ip.X/edge1.dx + b1);
    }
  } else
  {
    b1 = edge1.xbot - edge1.ybot * edge1.dx;
    b2 = edge2.xbot - edge2.ybot * edge2.dx;
    b2 = (b2-b1)/(edge1.dx - edge2.dx);
    ip.Y = Round(b2);
    ip.X = Round(edge1.dx * b2 + b1);
  }

  return
    //can be *so close* to the top of one edge that the rounded Y equals one ytop ...
    (ip.Y == edge1.ytop && ip.Y >= edge2.ytop && edge1.tmpX > edge2.tmpX) ||
    (ip.Y == edge2.ytop && ip.Y >= edge1.ytop && edge1.tmpX > edge2.tmpX) ||
    (ip.Y > edge1.ytop && ip.Y > edge2.ytop);
}
//------------------------------------------------------------------------------

void ReversePolyPtLinks(PolyPt &pp)
{
  PolyPt *pp1, *pp2;
  pp1 = &pp;
  do {
  pp2 = pp1->next;
  pp1->next = pp1->prev;
  pp1->prev = pp2;
  pp1 = pp2;
  } while( pp1 != &pp );
}
//------------------------------------------------------------------------------

void DisposePolyPts(PolyPt*& pp)
{
  if (pp == 0) return;
  PolyPt *tmpPp;
  pp->prev->next = 0;
  while( pp )
  {
    tmpPp = pp;
    pp = pp->next;
    delete tmpPp ;
  }
}
//------------------------------------------------------------------------------

bool IsClockwise(PolyPt *pt)
{
  long64 area = 0;
  PolyPt* startPt = pt;
  do
  {
    area = area + (long64)(pt->pt.X * pt->next->pt.Y) -
      (long64)(pt->next->pt.X * pt->pt.Y);
    pt = pt->next;
  }
  while (pt != startPt);
  //area = area /2;
  return area > 0; //ie reverse of normal formula because Y axis inverted
}
//------------------------------------------------------------------------------

inline bool PointsEqual( const IntPoint &pt1, const IntPoint &pt2)
{
  return ( pt1.X == pt2.X && pt1.Y == pt2.Y );
}
//------------------------------------------------------------------------------

void InitEdge(TEdge4 *e, TEdge4 *eNext,
  TEdge4 *ePrev, const IntPoint &pt, PolyType polyType)
{
  std::memset( e, 0, sizeof( TEdge4 ));

  e->next = eNext;
  e->prev = ePrev;
  e->xcurr = pt.X;
  e->ycurr = pt.Y;
  if (e->ycurr >= e->next->ycurr)
  {
    e->xbot = e->xcurr;
    e->ybot = e->ycurr;
    e->xtop = e->next->xcurr;
    e->ytop = e->next->ycurr;
    e->windDelta = 1;
  } else
  {
    e->xtop = e->xcurr;
    e->ytop = e->ycurr;
    e->xbot = e->next->xcurr;
    e->ybot = e->next->ycurr;
    e->windDelta = -1;
  }
  SetDx(*e);
  e->polyType = polyType;
  e->outIdx = -1;
}
//------------------------------------------------------------------------------

inline void SwapX(TEdge4 &e)
{
  //swap horizontal edges' top and bottom x's so they follow the natural
  //progression of the bounds - ie so their xbots will align with the
  //adjoining lower edge. [Helpful in the ProcessHorizontal() method.]
  e.xcurr = e.xtop;
  e.xtop = e.xbot;
  e.xbot = e.xcurr;
}
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Clipper4Base class methods ...
//------------------------------------------------------------------------------

Clipper4Base::Clipper4Base() //constructor
{
  m_MinimaList = 0;
  m_CurrentLM = 0;
}
//------------------------------------------------------------------------------

Clipper4Base::~Clipper4Base() //destructor
{
  Clear();
}
//------------------------------------------------------------------------------

void Clipper4Base::AddPolygon( const Polygon &pg, PolyType polyType)
{
  int len = pg.size();
  if (len < 3) return;
  Polygon p(len);
  p[0] = pg[0];
  int j = 0;
  for (int i = 1; i < len; ++i)
  {
    if (PointsEqual(p[j], pg[i])) continue;
    else if (j > 0 && SlopesEqual(p[j-1], p[j], pg[i]))
    {
      if (PointsEqual(p[j-1], pg[i])) j--;
    } else j++;
    p[j] = pg[i];
  }
  if (j < 2) return;

  len = j+1;
  for (;;)
  {
    //nb: test for point equality before testing slopes ...
    if (PointsEqual(p[j], p[0])) j--;
    else if (PointsEqual(p[0], p[1]) || SlopesEqual(p[j], p[0], p[1]))
      p[0] = p[j--];
    else if (SlopesEqual(p[j-1], p[j], p[0])) j--;
    else if (SlopesEqual(p[0], p[1], p[2]))
    {
      for (int i = 2; i <= j; ++i) p[i-1] = p[i];
      j--;
    }
    //exit loop if nothing is changed or there are too few vertices ...
    if (j == len-1 || j < 2) break;
    len = j +1;
  }
  if (len < 3) return;

  //create a new edge array ...
  TEdge4 *edges = new TEdge4 [len];
  m_edges.push_back(edges);

  //convert vertices to a double-linked-list of edges and initialize ...
  edges[0].xcurr = p[0].X;
  edges[0].ycurr = p[0].Y;
  InitEdge(&edges[len-1], &edges[0], &edges[len-2], p[len-1], polyType);
  for (int i = len-2; i > 0; --i)
    InitEdge(&edges[i], &edges[i+1], &edges[i-1], p[i], polyType);
  InitEdge(&edges[0], &edges[1], &edges[len-1], p[0], polyType);

  //reset xcurr & ycurr and find 'eHighest' (given the Y axis coordinates
  //increase downward so the 'highest' edge will have the smallest ytop) ...
  TEdge4 *e = &edges[0];
  TEdge4 *eHighest = e;
  do
  {
    e->xcurr = e->xbot;
    e->ycurr = e->ybot;
    if (e->ytop < eHighest->ytop) eHighest = e;
    e = e->next;
  }
  while ( e != &edges[0]);

  //make sure eHighest is positioned so the following loop works safely ...
  if (eHighest->windDelta > 0) eHighest = eHighest->next;
  if (eHighest->dx == horizontal) eHighest = eHighest->next;

  //finally insert each local minima ...
  e = eHighest;
  do {
    e = AddBoundsToLML(e);
  }
  while( e != eHighest );
}
//------------------------------------------------------------------------------

void Clipper4Base::InsertLocalMinima(LocalMinima *newLm)
{
  if( ! m_MinimaList )
  {
    m_MinimaList = newLm;
  }
  else if( newLm->Y >= m_MinimaList->Y )
  {
    newLm->next = m_MinimaList;
    m_MinimaList = newLm;
  } else
  {
    LocalMinima* tmpLm = m_MinimaList;
    while( tmpLm->next  && ( newLm->Y < tmpLm->next->Y ) )
      tmpLm = tmpLm->next;
    newLm->next = tmpLm->next;
    tmpLm->next = newLm;
  }
}
//------------------------------------------------------------------------------

TEdge4* Clipper4Base::AddBoundsToLML(TEdge4 *e)
{
  //Starting at the top of one bound we progress to the bottom where there's
  //a local minima. We then go to the top of the next bound. These two bounds
  //form the left and right (or right and left) bounds of the local minima.
  e->nextInLML = 0;
  e = e->next;
  for (;;)
  {
    if ( e->dx == horizontal )
    {
      //nb: proceed through horizontals when approaching from their right,
      //    but break on horizontal minima if approaching from their left.
      //    This ensures 'local minima' are always on the left of horizontals.
      if (e->next->ytop < e->ytop && e->next->xbot > e->prev->xbot) break;
      if (e->xtop != e->prev->xbot) SwapX(*e);
      e->nextInLML = e->prev;
    }
    else if (e->ycurr == e->prev->ycurr) break;
    else e->nextInLML = e->prev;
    e = e->next;
  }

  //e and e.prev are now at a local minima ...
  LocalMinima* newLm = new LocalMinima;
  newLm->next = 0;
  newLm->Y = e->prev->ybot;

  if ( e->dx == horizontal ) //horizontal edges never start a left bound
  {
    if (e->xbot != e->prev->xbot) SwapX(*e);
    newLm->leftBound = e->prev;
    newLm->rightBound = e;
  } else if (e->dx < e->prev->dx)
  {
    newLm->leftBound = e->prev;
    newLm->rightBound = e;
  } else
  {
    newLm->leftBound = e;
    newLm->rightBound = e->prev;
  }
  newLm->leftBound->side = esLeft;
  newLm->rightBound->side = esRight;
  InsertLocalMinima( newLm );

  for (;;)
  {
    if ( e->next->ytop == e->ytop && e->next->dx != horizontal ) break;
    e->nextInLML = e->next;
    e = e->next;
    if ( e->dx == horizontal && e->xbot != e->prev->xtop) SwapX(*e);
  }
  return e->next;
}
//------------------------------------------------------------------------------

void Clipper4Base::AddPolygons(const Polygons &ppg, PolyType polyType)
{
  for (Polygons::size_type i = 0; i < ppg.size(); ++i)
    AddPolygon(ppg[i], polyType);
}
//------------------------------------------------------------------------------

void Clipper4Base::Clear()
{
  DisposeLocalMinimaList();
  for (EdgeList::size_type i = 0; i < m_edges.size(); ++i) delete [] m_edges[i];
  m_edges.clear();
}
//------------------------------------------------------------------------------

bool Clipper4Base::Reset()
{
  m_CurrentLM = m_MinimaList;
  if( !m_CurrentLM ) return false; //ie nothing to process

  //reset all edges ...
  LocalMinima* lm = m_MinimaList;
  while( lm )
  {
    TEdge4* e = lm->leftBound;
    while( e )
    {
      e->xcurr = e->xbot;
      e->ycurr = e->ybot;
      e->side = esLeft;
      e->outIdx = -1;
      e = e->nextInLML;
    }
    e = lm->rightBound;
    while( e )
    {
      e->xcurr = e->xbot;
      e->ycurr = e->ybot;
      e->side = esRight;
      e->outIdx = -1;
      e = e->nextInLML;
    }
    lm = lm->next;
  }
  return true;
}
//------------------------------------------------------------------------------

void Clipper4Base::DisposeLocalMinimaList()
{
  while( m_MinimaList )
  {
    LocalMinima* tmpLm = m_MinimaList->next;
    delete m_MinimaList;
    m_MinimaList = tmpLm;
  }
  m_CurrentLM = 0;
}
//------------------------------------------------------------------------------

void Clipper4Base::PopLocalMinima()
{
  if( ! m_CurrentLM ) return;
  m_CurrentLM = m_CurrentLM->next;
}

//------------------------------------------------------------------------------
// TClipper4 methods ...
//------------------------------------------------------------------------------

Clipper4::Clipper4() : Clipper4Base() //constructor
{
  m_Scanbeam = 0;
  m_ActiveEdges = 0;
  m_SortedEdges = 0;
  m_IntersectNodes = 0;
  m_ExecuteLocked = false;
};
//------------------------------------------------------------------------------

Clipper4::~Clipper4() //destructor
{
  DisposeScanbeamList();
};
//------------------------------------------------------------------------------

void Clipper4::DisposeScanbeamList()
{
  while ( m_Scanbeam ) {
  Scanbeam* sb2 = m_Scanbeam->next;
  delete m_Scanbeam;
  m_Scanbeam = sb2;
  }
}
//------------------------------------------------------------------------------

bool Clipper4::Reset()
{
  if (!Clipper4Base::Reset()) return false;
  m_Scanbeam = 0;
  m_ActiveEdges = 0;
  m_SortedEdges = 0;
  LocalMinima* lm = m_MinimaList;
  while (lm)
  {
    InsertScanbeam(lm->Y);
    InsertScanbeam(lm->leftBound->ytop);
    lm = lm->next;
  }
  return true;
}
//------------------------------------------------------------------------------

bool Clipper4::Execute(ClipType clipType, Polygons &solution,
    PolyFillType subjFillType, PolyFillType clipFillType)
{
  if( m_ExecuteLocked ) return false;
  bool succeeded;
  try {
    m_ExecuteLocked = true;
    solution.resize(0);
    if ( !Reset() )
    {
      m_ExecuteLocked = false;
      return false;
    }
    m_SubjFillType = subjFillType;
    m_ClipFillType = clipFillType;
    m_ClipType = clipType;

    int botY = PopScanbeam();
    do {
      InsertLocalMinimaIntoAEL(botY);
      ProcessHorizontals();
      int topY = PopScanbeam();
      succeeded = ProcessIntersections(topY);
      if (succeeded) ProcessEdgesAtTopOfScanbeam(topY);
      botY = topY;
    } while( succeeded && m_Scanbeam );

    //build the return polygons ...
    if (succeeded) BuildResult(solution);
  }
  catch(...) {
    solution.resize(0);
    succeeded = false;
  }
  DisposeAllPolyPts();
  m_ExecuteLocked = false;
  return succeeded;
}
//------------------------------------------------------------------------------

void Clipper4::InsertScanbeam(const int Y)
{
  if( !m_Scanbeam )
  {
    m_Scanbeam = new Scanbeam;
    m_Scanbeam->next = 0;
    m_Scanbeam->Y = Y;
  }
  else if(  Y > m_Scanbeam->Y )
  {
    Scanbeam* newSb = new Scanbeam;
    newSb->Y = Y;
    newSb->next = m_Scanbeam;
    m_Scanbeam = newSb;
  } else
  {
    Scanbeam* sb2 = m_Scanbeam;
    while( sb2->next  && ( Y <= sb2->next->Y ) ) sb2 = sb2->next;
    if(  Y == sb2->Y ) return; //ie ignores duplicates
    Scanbeam* newSb = new Scanbeam;
    newSb->Y = Y;
    newSb->next = sb2->next;
    sb2->next = newSb;
  }
}
//------------------------------------------------------------------------------

int Clipper4::PopScanbeam()
{
  int Y = m_Scanbeam->Y;
  Scanbeam* sb2 = m_Scanbeam;
  m_Scanbeam = m_Scanbeam->next;
  delete sb2;
  return Y;
}
//------------------------------------------------------------------------------

void Clipper4::DisposeAllPolyPts(){
  for (PolyPtList::size_type i = 0; i < m_PolyPts.size(); ++i)
    DisposePolyPts(m_PolyPts[i]);
  m_PolyPts.clear();
}
//------------------------------------------------------------------------------

void Clipper4::SetWindingCount(TEdge4 &edge)
{
  TEdge4 *e = edge.prevInAEL;
  //find the edge of the same polytype that immediately preceeds 'edge' in AEL
  while ( e  && e->polyType != edge.polyType ) e = e->prevInAEL;
  if ( !e )
  {
    edge.windCnt = edge.windDelta;
    edge.windCnt2 = 0;
    e = m_ActiveEdges; //ie get ready to calc windCnt2
  } else if ( IsNonZeroFillType(edge) )
  {
    //nonZero filling ...
    if ( e->windCnt * e->windDelta < 0 )
    {
      if (std::abs(e->windCnt) > 1)
      {
        if (e->windDelta * edge.windDelta < 0) edge.windCnt = e->windCnt;
        else edge.windCnt = e->windCnt + edge.windDelta;
      } else
        edge.windCnt = e->windCnt + e->windDelta + edge.windDelta;
    } else
    {
      if ( std::abs(e->windCnt) > 1 && e->windDelta * edge.windDelta < 0)
        edge.windCnt = e->windCnt;
      else if ( e->windCnt + edge.windDelta == 0 )
        edge.windCnt = e->windCnt;
      else edge.windCnt = e->windCnt + edge.windDelta;
    }
    edge.windCnt2 = e->windCnt2;
    e = e->nextInAEL; //ie get ready to calc windCnt2
  } else
  {
    //even-odd filling ...
    edge.windCnt = 1;
    edge.windCnt2 = e->windCnt2;
    e = e->nextInAEL; //ie get ready to calc windCnt2
  }

  //update windCnt2 ...
  if ( IsNonZeroAltFillType(edge) )
  {
    //nonZero filling ...
    while ( e != &edge )
    {
      edge.windCnt2 += e->windDelta;
      e = e->nextInAEL;
    }
  } else
  {
    //even-odd filling ...
    while ( e != &edge )
    {
      edge.windCnt2 = (edge.windCnt2 == 0) ? 1 : 0;
      e = e->nextInAEL;
    }
  }
}
//------------------------------------------------------------------------------

bool Clipper4::IsNonZeroFillType(const TEdge4& edge) const
{
  if (edge.polyType == ptSubject)
    return m_SubjFillType == pftNonZero; else
    return m_ClipFillType == pftNonZero;
}
//------------------------------------------------------------------------------

bool Clipper4::IsNonZeroAltFillType(const TEdge4& edge) const
{
  if (edge.polyType == ptSubject)
    return m_ClipFillType == pftNonZero; else
    return m_SubjFillType == pftNonZero;
}
//------------------------------------------------------------------------------

bool Clipper4::IsContributing(const TEdge4& edge) const
{
  switch( m_ClipType ){
    case ctIntersection:
      if ( edge.polyType == ptSubject )
        return std::abs(edge.windCnt) == 1 && edge.windCnt2 != 0; else
        return std::abs(edge.windCnt2) > 0 && std::abs(edge.windCnt) == 1;
    case ctUnion:
      return std::abs(edge.windCnt) == 1 && edge.windCnt2 == 0;
    case ctDifference:
      if ( edge.polyType == ptSubject )
        return std::abs(edge.windCnt) == 1 && edge.windCnt2 == 0; else
        return std::abs(edge.windCnt) == 1 && edge.windCnt2 != 0;
    default: //case ctXor:
      return std::abs(edge.windCnt) == 1;
  }
}
//------------------------------------------------------------------------------

void Clipper4::AddLocalMinPoly(TEdge4 *e1, TEdge4 *e2, const IntPoint &pt)
{
  AddPolyPt( e1, pt );
  e2->outIdx = e1->outIdx;

  if( e2->dx == horizontal || ( e1->dx > e2->dx ) )
  {
    e1->side = esLeft;
    e2->side = esRight;
  } else
  {
    e1->side = esRight;
    e2->side = esLeft;
  }
}
//------------------------------------------------------------------------------

void Clipper4::AddLocalMaxPoly(TEdge4 *e1, TEdge4 *e2, const IntPoint &pt)
{
  AddPolyPt( e1, pt );
  if( e1->outIdx == e2->outIdx )
  {
    e1->outIdx = -1;
    e2->outIdx = -1;
  }
  else
    AppendPolygon( e1, e2 );
}
//------------------------------------------------------------------------------

void Clipper4::AddEdgeToSEL(TEdge4 *edge)
{
  //SEL pointers in PEdge are reused to build a list of horizontal edges.
  //However, we don't need to worry about order with horizontal edge processing.
  if( !m_SortedEdges )
  {
    m_SortedEdges = edge;
    edge->prevInSEL = 0;
    edge->nextInSEL = 0;
  }
  else
  {
    edge->nextInSEL = m_SortedEdges;
    edge->prevInSEL = 0;
    m_SortedEdges->prevInSEL = edge;
    m_SortedEdges = edge;
  }
}
//------------------------------------------------------------------------------

void Clipper4::CopyAELToSEL()
{
  TEdge4* e = m_ActiveEdges;
  m_SortedEdges = e;
  if (!m_ActiveEdges) return;
  m_SortedEdges->prevInSEL = 0;
  e = e->nextInAEL;
  while ( e )
  {
    e->prevInSEL = e->prevInAEL;
    e->prevInSEL->nextInSEL = e;
    e->nextInSEL = 0;
    e = e->nextInAEL;
  }
}
//------------------------------------------------------------------------------

void Clipper4::InsertLocalMinimaIntoAEL( const int botY)
{
  while(  m_CurrentLM  && ( m_CurrentLM->Y == botY ) )
  {
    TEdge4* lb = m_CurrentLM->leftBound;
    TEdge4* rb = m_CurrentLM->rightBound;

    InsertEdgeIntoAEL( lb );
    InsertScanbeam( lb->ytop );
    InsertEdgeIntoAEL( rb );

    if ( IsNonZeroFillType( *lb) )
      rb->windDelta = -lb->windDelta;
    else
    {
      lb->windDelta = 1;
      rb->windDelta = 1;
    }
    SetWindingCount( *lb );
    rb->windCnt = lb->windCnt;
    rb->windCnt2 = lb->windCnt2;

    if(  rb->dx == horizontal )
    {
      //nb: only rightbounds can have a horizontal bottom edge
      AddEdgeToSEL( rb );
      InsertScanbeam( rb->nextInLML->ytop );
    }
    else
      InsertScanbeam( rb->ytop );

    if( IsContributing(*lb) )
      AddLocalMinPoly( lb, rb, MakePoint(lb->xcurr, m_CurrentLM->Y) );

    if( lb->nextInAEL != rb )
    {
      TEdge4* e = lb->nextInAEL;
      IntPoint pt = MakePoint(lb->xcurr, lb->ycurr);
      while( e != rb )
      {
        if(!e) throw clipper4Exception("InsertLocalMinimaIntoAEL: missing rightbound!");
        //nb: For calculating winding counts etc, IntersectEdges() assumes
        //that param1 will be to the right of param2 ABOVE the intersection ...
        IntersectEdges( rb , e , pt , ipNone); //order important here
        e = e->nextInAEL;
      }
    }
    PopLocalMinima();
  }
}
//------------------------------------------------------------------------------

void Clipper4::DeleteFromAEL(TEdge4 *e)
{
  TEdge4* AelPrev = e->prevInAEL;
  TEdge4* AelNext = e->nextInAEL;
  if(  !AelPrev &&  !AelNext && (e != m_ActiveEdges) ) return; //already deleted
  if( AelPrev ) AelPrev->nextInAEL = AelNext;
  else m_ActiveEdges = AelNext;
  if( AelNext ) AelNext->prevInAEL = AelPrev;
  e->nextInAEL = 0;
  e->prevInAEL = 0;
}
//------------------------------------------------------------------------------

void Clipper4::DeleteFromSEL(TEdge4 *e)
{
  TEdge4* SelPrev = e->prevInSEL;
  TEdge4* SelNext = e->nextInSEL;
  if(  !SelPrev &&  !SelNext && (e != m_SortedEdges) ) return; //already deleted
  if( SelPrev ) SelPrev->nextInSEL = SelNext;
  else m_SortedEdges = SelNext;
  if( SelNext ) SelNext->prevInSEL = SelPrev;
  e->nextInSEL = 0;
  e->prevInSEL = 0;
}
//------------------------------------------------------------------------------

void Clipper4::IntersectEdges(TEdge4 *e1, TEdge4 *e2,
     const IntPoint &pt, IntersectProtects protects)
{
  //e1 will be to the left of e2 BELOW the intersection. Therefore e1 is before
  //e2 in AEL except when e1 is being inserted at the intersection point ...
  bool e1stops = !(ipLeft & protects) &&  !e1->nextInLML &&
    e1->xtop == pt.X && e1->ytop == pt.Y;
  bool e2stops = !(ipRight & protects) &&  !e2->nextInLML &&
    e2->xtop == pt.X && e2->ytop == pt.Y;
  bool e1Contributing = ( e1->outIdx >= 0 );
  bool e2contributing = ( e2->outIdx >= 0 );

  //update winding counts...
  //assumes that e1 will be to the right of e2 ABOVE the intersection
  if ( e1->polyType == e2->polyType )
  {
    if ( IsNonZeroFillType( *e1) )
    {
      if (e1->windCnt + e2->windDelta == 0 ) e1->windCnt = -e1->windCnt;
      else e1->windCnt += e2->windDelta;
      if ( e2->windCnt - e1->windDelta == 0 ) e2->windCnt = -e2->windCnt;
      else e2->windCnt -= e1->windDelta;
    } else
    {
      int oldE1WindCnt = e1->windCnt;
      e1->windCnt = e2->windCnt;
      e2->windCnt = oldE1WindCnt;
    }
  } else
  {
    if ( IsNonZeroFillType(*e2) ) e1->windCnt2 += e2->windDelta;
    else e1->windCnt2 = ( e1->windCnt2 == 0 ) ? 1 : 0;
    if ( IsNonZeroFillType(*e1) ) e2->windCnt2 -= e1->windDelta;
    else e2->windCnt2 = ( e2->windCnt2 == 0 ) ? 1 : 0;
  }

  if ( e1Contributing && e2contributing )
  {
    if ( e1stops || e2stops || std::abs(e1->windCnt) > 1 ||
      std::abs(e2->windCnt) > 1 ||
      (e1->polyType != e2->polyType && m_ClipType != ctXor) )
        AddLocalMaxPoly(e1, e2, pt); else
        DoBothEdges( e1, e2, pt );
  }
  else if ( e1Contributing )
  {
    switch( m_ClipType ) {
      case ctIntersection:
        if ( (e2->polyType == ptSubject || e2->windCnt2 != 0) &&
           std::abs(e2->windCnt) < 2 ) DoEdge1( e1, e2, pt);
        break;
      default:
        if ( std::abs(e2->windCnt) < 2 ) DoEdge1(e1, e2, pt);
    }
  }
  else if ( e2contributing )
  {
    if ( m_ClipType == ctIntersection )
    {
        if ( (e1->polyType == ptSubject || e1->windCnt2 != 0) &&
          std::abs(e1->windCnt) < 2 ) DoEdge2( e1, e2, pt );
    }
    else
      if (std::abs(e1->windCnt) < 2) DoEdge2( e1, e2, pt );

  } else
  {
    //neither edge is currently contributing ...
    if ( std::abs(e1->windCnt) > 1 && std::abs(e2->windCnt) > 1 ) ;// do nothing
    else if ( e1->polyType != e2->polyType && !e1stops && !e2stops &&
      std::abs(e1->windCnt) < 2 && std::abs(e2->windCnt) < 2 )
        AddLocalMinPoly(e1, e2, pt);
    else if ( std::abs(e1->windCnt) == 1 && std::abs(e2->windCnt) == 1 )
      switch( m_ClipType ) {
        case ctIntersection:
          if ( std::abs(e1->windCnt2) > 0 && std::abs(e2->windCnt2) > 0 )
            AddLocalMinPoly(e1, e2, pt);
          break;
        case ctUnion:
          if ( e1->windCnt2 == 0 && e2->windCnt2 == 0 )
            AddLocalMinPoly(e1, e2, pt);
          break;
        case ctDifference:
          if ( (e1->polyType == ptClip && e2->polyType == ptClip &&
            e1->windCnt2 != 0 && e2->windCnt2 != 0) ||
            (e1->polyType == ptSubject && e2->polyType == ptSubject &&
            e1->windCnt2 == 0 && e2->windCnt2 == 0) )
              AddLocalMinPoly(e1, e2, pt);
          break;
        case ctXor:
          AddLocalMinPoly(e1, e2, pt);
      }
    else if ( std::abs(e1->windCnt) < 2 && std::abs(e2->windCnt) < 2 )
      SwapSides( *e1, *e2 );
  }

  if(  (e1stops != e2stops) &&
    ( (e1stops && (e1->outIdx >= 0)) || (e2stops && (e2->outIdx >= 0)) ) )
  {
    SwapSides( *e1, *e2 );
    SwapPolyIndexes( *e1, *e2 );
  }

  //finally, delete any non-contributing maxima edges  ...
  if( e1stops ) DeleteFromAEL( e1 );
  if( e2stops ) DeleteFromAEL( e2 );
}
//------------------------------------------------------------------------------

int PolygonBottom(PolyPt *pp)
{
  int result = pp->pt.Y;
  PolyPt *p = pp->next;
  while (p != pp)
  {
    if (p->pt.Y > result) result = p->pt.Y;
    p = p->next;
  }
  return result;
}
//------------------------------------------------------------------------------

void Clipper4::AppendPolygon(TEdge4 *e1, TEdge4 *e2)
{
  //get the start and ends of both output polygons ...
  PolyPt* p1_lft = m_PolyPts[e1->outIdx];
  PolyPt* p1_rt = p1_lft->prev;
  PolyPt* p2_lft = m_PolyPts[e2->outIdx];
  PolyPt* p2_rt = p2_lft->prev;

  //fixup orientation (hole) flag if necessary ...
  if (p1_lft->isHole != p2_lft->isHole)
  {
    PolyPt *p, *pp;
    if (PolygonBottom(p1_lft) < PolygonBottom(p2_lft))
      p = p1_lft; else p = p2_lft;
    bool hole = !p->isHole;
    pp = p;
    do
    {
      pp->isHole = hole;
      pp = pp->next;
    }
    while (pp != p);
  }

  EdgeSide side;
  //join e2 poly onto e1 poly and delete pointers to e2 ...
  if(  e1->side == esLeft )
  {
    if(  e2->side == esLeft )
    {
      //z y x a b c
      ReversePolyPtLinks(*p2_lft);
      p2_lft->next = p1_lft;
      p1_lft->prev = p2_lft;
      p1_rt->next = p2_rt;
      p2_rt->prev = p1_rt;
      m_PolyPts[e1->outIdx] = p2_rt;
    } else
    {
      //x y z a b c
      p2_rt->next = p1_lft;
      p1_lft->prev = p2_rt;
      p2_lft->prev = p1_rt;
      p1_rt->next = p2_lft;
      m_PolyPts[e1->outIdx] = p2_lft;
    }
    side = esLeft;
  } else
  {
    if(  e2->side == esRight )
    {
      //a b c z y x
      ReversePolyPtLinks( *p2_lft );
      p1_rt->next = p2_rt;
      p2_rt->prev = p1_rt;
      p2_lft->next = p1_lft;
      p1_lft->prev = p2_lft;
    } else
    {
      //a b c x y z
      p1_rt->next = p2_lft;
      p2_lft->prev = p1_rt;
      p1_lft->prev = p2_rt;
      p2_rt->next = p1_lft;
    }
    side = esRight;
  }

  int OKIdx = e1->outIdx;
  int ObsoleteIdx = e2->outIdx;
  m_PolyPts[ObsoleteIdx] = 0;

  e1->outIdx = -1; //nb: safe because we only get here via AddLocalMaxPoly
  e2->outIdx = -1;

  TEdge4* e = m_ActiveEdges;
  while( e )
  {
    if( e->outIdx == ObsoleteIdx )
    {
      e->outIdx = OKIdx;
      e->side = side;
      break;
    }
    e = e->nextInAEL;
  }
}
//------------------------------------------------------------------------------

PolyPt* Clipper4::AddPolyPt(TEdge4 *e, const IntPoint &pt)
{
  bool ToFront = (e->side == esLeft);
  if(  e->outIdx < 0 )
  {
    PolyPt* newPolyPt = new PolyPt;
    newPolyPt->pt = pt;
    newPolyPt->isHole = IsHole(e);
    m_PolyPts.push_back(newPolyPt);
    newPolyPt->next = newPolyPt;
    newPolyPt->prev = newPolyPt;
    e->outIdx = m_PolyPts.size()-1;
    return newPolyPt;
  } else
  {
    PolyPt* pp = m_PolyPts[e->outIdx];
    if (ToFront && PointsEqual(pt, pp->pt)) return pp;
    if (!ToFront && PointsEqual(pt, pp->prev->pt)) return pp->prev;

    PolyPt* newPolyPt = new PolyPt;
    newPolyPt->pt = pt;
    newPolyPt->isHole = pp->isHole;
    newPolyPt->next = pp;
    newPolyPt->prev = pp->prev;
    newPolyPt->prev->next = newPolyPt;
    pp->prev = newPolyPt;
    if (ToFront) m_PolyPts[e->outIdx] = newPolyPt;
    return newPolyPt;
  }
}
//------------------------------------------------------------------------------

void Clipper4::ProcessHorizontals()
{
  TEdge4* horzEdge = m_SortedEdges;
  while( horzEdge )
  {
    DeleteFromSEL( horzEdge );
    ProcessHorizontal( horzEdge );
    horzEdge = m_SortedEdges;
  }
}
//------------------------------------------------------------------------------

bool Clipper4::IsTopHorz(const int XPos)
{
  TEdge4* e = m_SortedEdges;
  while( e )
  {
    if(  ( XPos >= std::min(e->xcurr, e->xtop) ) &&
      ( XPos <= std::max(e->xcurr, e->xtop) ) ) return false;
    e = e->nextInSEL;
  }
  return true;
}
//------------------------------------------------------------------------------

bool IsMinima(TEdge4 *e)
{
  return e  && (e->prev->nextInLML != e) && (e->next->nextInLML != e);
}
//------------------------------------------------------------------------------

bool IsMaxima(TEdge4 *e, const int Y)
{
  return e && e->ytop == Y && !e->nextInLML;
}
//------------------------------------------------------------------------------

bool IsIntermediate(TEdge4 *e, const int Y)
{
  return e->ytop == Y && e->nextInLML;
}
//------------------------------------------------------------------------------

TEdge4 *GetMaximaPair(TEdge4 *e)
{
  if( !IsMaxima(e->next, e->ytop) || (e->next->xtop != e->xtop) )
    return e->prev; else
    return e->next;
}
//------------------------------------------------------------------------------

void Clipper4::SwapPositionsInAEL(TEdge4 *edge1, TEdge4 *edge2)
{
  if(  !( edge1->nextInAEL ) &&  !( edge1->prevInAEL ) ) return;
  if(  !( edge2->nextInAEL ) &&  !( edge2->prevInAEL ) ) return;

  if(  edge1->nextInAEL == edge2 )
  {
    TEdge4* next = edge2->nextInAEL;
    if( next ) next->prevInAEL = edge1;
    TEdge4* prev = edge1->prevInAEL;
    if( prev ) prev->nextInAEL = edge2;
    edge2->prevInAEL = prev;
    edge2->nextInAEL = edge1;
    edge1->prevInAEL = edge2;
    edge1->nextInAEL = next;
  }
  else if(  edge2->nextInAEL == edge1 )
  {
    TEdge4* next = edge1->nextInAEL;
    if( next ) next->prevInAEL = edge2;
    TEdge4* prev = edge2->prevInAEL;
    if( prev ) prev->nextInAEL = edge1;
    edge1->prevInAEL = prev;
    edge1->nextInAEL = edge2;
    edge2->prevInAEL = edge1;
    edge2->nextInAEL = next;
  }
  else
  {
    TEdge4* next = edge1->nextInAEL;
    TEdge4* prev = edge1->prevInAEL;
    edge1->nextInAEL = edge2->nextInAEL;
    if( edge1->nextInAEL ) edge1->nextInAEL->prevInAEL = edge1;
    edge1->prevInAEL = edge2->prevInAEL;
    if( edge1->prevInAEL ) edge1->prevInAEL->nextInAEL = edge1;
    edge2->nextInAEL = next;
    if( edge2->nextInAEL ) edge2->nextInAEL->prevInAEL = edge2;
    edge2->prevInAEL = prev;
    if( edge2->prevInAEL ) edge2->prevInAEL->nextInAEL = edge2;
  }

  if( !edge1->prevInAEL ) m_ActiveEdges = edge1;
  else if( !edge2->prevInAEL ) m_ActiveEdges = edge2;
}
//------------------------------------------------------------------------------

void Clipper4::SwapPositionsInSEL(TEdge4 *edge1, TEdge4 *edge2)
{
  if(  !( edge1->nextInSEL ) &&  !( edge1->prevInSEL ) ) return;
  if(  !( edge2->nextInSEL ) &&  !( edge2->prevInSEL ) ) return;

  if(  edge1->nextInSEL == edge2 )
  {
    TEdge4* next = edge2->nextInSEL;
    if( next ) next->prevInSEL = edge1;
    TEdge4* prev = edge1->prevInSEL;
    if( prev ) prev->nextInSEL = edge2;
    edge2->prevInSEL = prev;
    edge2->nextInSEL = edge1;
    edge1->prevInSEL = edge2;
    edge1->nextInSEL = next;
  }
  else if(  edge2->nextInSEL == edge1 )
  {
    TEdge4* next = edge1->nextInSEL;
    if( next ) next->prevInSEL = edge2;
    TEdge4* prev = edge2->prevInSEL;
    if( prev ) prev->nextInSEL = edge1;
    edge1->prevInSEL = prev;
    edge1->nextInSEL = edge2;
    edge2->prevInSEL = edge1;
    edge2->nextInSEL = next;
  }
  else
  {
    TEdge4* next = edge1->nextInSEL;
    TEdge4* prev = edge1->prevInSEL;
    edge1->nextInSEL = edge2->nextInSEL;
    if( edge1->nextInSEL ) edge1->nextInSEL->prevInSEL = edge1;
    edge1->prevInSEL = edge2->prevInSEL;
    if( edge1->prevInSEL ) edge1->prevInSEL->nextInSEL = edge1;
    edge2->nextInSEL = next;
    if( edge2->nextInSEL ) edge2->nextInSEL->prevInSEL = edge2;
    edge2->prevInSEL = prev;
    if( edge2->prevInSEL ) edge2->prevInSEL->nextInSEL = edge2;
  }

  if( !edge1->prevInSEL ) m_SortedEdges = edge1;
  else if( !edge2->prevInSEL ) m_SortedEdges = edge2;
}
//------------------------------------------------------------------------------

TEdge4* GetNextInAEL(TEdge4 *e, Direction dir)
{
  if( dir == dLeftToRight ) return e->nextInAEL;
  else return e->prevInAEL;
}
//------------------------------------------------------------------------------

void Clipper4::ProcessHorizontal(TEdge4 *horzEdge)
{
  Direction dir;
  int horzLeft, horzRight;

  if( horzEdge->xcurr < horzEdge->xtop )
  {
    horzLeft = horzEdge->xcurr;
    horzRight = horzEdge->xtop;
    dir = dLeftToRight;
  } else
  {
    horzLeft = horzEdge->xtop;
    horzRight = horzEdge->xcurr;
    dir = dRightToLeft;
  }

  TEdge4* eMaxPair;
  if( horzEdge->nextInLML ) eMaxPair = 0;
  else eMaxPair = GetMaximaPair(horzEdge);

  TEdge4* e = GetNextInAEL( horzEdge , dir );
  while( e )
  {
    TEdge4* eNext = GetNextInAEL( e, dir );
    if( e->xcurr >= horzLeft && e->xcurr <= horzRight )
    {
      //ok, so far it looks like we're still in range of the horizontal edge
      if ( e->xcurr == horzEdge->xtop && horzEdge->nextInLML &&
        e->dx <= horzEdge->nextInLML->dx) break;

      if( e == eMaxPair )
      {
        //horzEdge is evidently a maxima horizontal and we've arrived at its end.
        if (dir == dLeftToRight)
          IntersectEdges(horzEdge, e, MakePoint(e->xcurr, horzEdge->ycurr), ipNone);
        else
          IntersectEdges(e, horzEdge, MakePoint(e->xcurr, horzEdge->ycurr), ipNone);
        return;
      }
      else if( e->dx == horizontal &&  !IsMinima(e) && !(e->xcurr > e->xtop) )
      {
        //An overlapping horizontal edge. Overlapping horizontal edges are
        //processed as if layered with the current horizontal edge (horizEdge)
        //being infinitesimally lower that the next (e). Therfore, we
        //intersect with e only if e.xcurr is within the bounds of horzEdge ...
        if( dir == dLeftToRight )
          IntersectEdges( horzEdge , e, MakePoint(e->xcurr, horzEdge->ycurr),
            (IsTopHorz( e->xcurr ))? ipLeft : ipBoth );
        else
          IntersectEdges( e, horzEdge, MakePoint(e->xcurr, horzEdge->ycurr),
            (IsTopHorz( e->xcurr ))? ipRight : ipBoth );
      }
      else if( dir == dLeftToRight )
      {
        IntersectEdges( horzEdge, e, MakePoint(e->xcurr, horzEdge->ycurr),
          (IsTopHorz( e->xcurr ))? ipLeft : ipBoth );
      }
      else
      {
        IntersectEdges( e, horzEdge, MakePoint(e->xcurr, horzEdge->ycurr),
          (IsTopHorz( e->xcurr ))? ipRight : ipBoth );
      }
      SwapPositionsInAEL( horzEdge, e );
    }
    else if( dir == dLeftToRight &&
      e->xcurr > horzRight  && m_SortedEdges ) break;
    else if( dir == dRightToLeft &&
      e->xcurr < horzLeft && m_SortedEdges ) break;
    e = eNext;
  } //end while

  if( horzEdge->nextInLML )
  {
    if( horzEdge->outIdx >= 0 )
      AddPolyPt( horzEdge, MakePoint(horzEdge->xtop, horzEdge->ytop));
    UpdateEdgeIntoAEL( horzEdge );
  }
  else
  {
    if ( horzEdge->outIdx >= 0 )
      IntersectEdges( horzEdge, eMaxPair,
      MakePoint(horzEdge->xtop, horzEdge->ycurr), ipBoth);
    if (eMaxPair->outIdx >= 0) throw clipper4Exception("ProcessHorizontal error");
    DeleteFromAEL(eMaxPair);
    DeleteFromAEL(horzEdge);
  }
}
//------------------------------------------------------------------------------

void Clipper4::UpdateEdgeIntoAEL(TEdge4 *&e)
{
  if( !e->nextInLML ) throw
    clipper4Exception("UpdateEdgeIntoAEL: invalid call");
  TEdge4* AelPrev = e->prevInAEL;
  TEdge4* AelNext = e->nextInAEL;
  e->nextInLML->outIdx = e->outIdx;
  if( AelPrev ) AelPrev->nextInAEL = e->nextInLML;
  else m_ActiveEdges = e->nextInLML;
  if( AelNext ) AelNext->prevInAEL = e->nextInLML;
  e->nextInLML->side = e->side;
  e->nextInLML->windDelta = e->windDelta;
  e->nextInLML->windCnt = e->windCnt;
  e->nextInLML->windCnt2 = e->windCnt2;
  e = e->nextInLML;
  e->prevInAEL = AelPrev;
  e->nextInAEL = AelNext;
  if( e->dx != horizontal ) InsertScanbeam( e->ytop );
}
//------------------------------------------------------------------------------

bool Clipper4::ProcessIntersections( const int topY)
{
  if( !m_ActiveEdges ) return true;
  try {
    BuildIntersectList(topY);
    if ( !m_IntersectNodes) return true;
    if ( FixupIntersections() ) ProcessIntersectList();
    else return false;
  }
  catch(...) {
    m_SortedEdges = 0;
    DisposeIntersectNodes();
    throw clipper4Exception("ProcessIntersections error");
  }
  return true;
}
//------------------------------------------------------------------------------

void Clipper4::DisposeIntersectNodes()
{
  while ( m_IntersectNodes )
  {
    IntersectNode* iNode = m_IntersectNodes->next;
    delete m_IntersectNodes;
    m_IntersectNodes = iNode;
  }
}
//------------------------------------------------------------------------------

void Clipper4::BuildIntersectList(const int topY)
{
  if ( !m_ActiveEdges ) return;

  //prepare for sorting ...
  TEdge4* e = m_ActiveEdges;
  e->tmpX = TopX( *e, topY );
  m_SortedEdges = e;
  m_SortedEdges->prevInSEL = 0;
  e = e->nextInAEL;
  while( e )
  {
    e->prevInSEL = e->prevInAEL;
    e->prevInSEL->nextInSEL = e;
    e->nextInSEL = 0;
    e->tmpX = TopX( *e, topY );
    e = e->nextInAEL;
  }

  //bubblesort ...
  bool isModified = true;
  while( isModified && m_SortedEdges )
  {
    isModified = false;
    e = m_SortedEdges;
    while( e->nextInSEL )
    {
      TEdge4 *eNext = e->nextInSEL;
      IntPoint pt;
      if(e->tmpX > eNext->tmpX && IntersectPoint(*e, *eNext, pt))
      {
        AddIntersectNode( e, eNext, pt );
        SwapPositionsInSEL(e, eNext);
        isModified = true;
      }
      else
        e = eNext;
    }
    if( e->prevInSEL ) e->prevInSEL->nextInSEL = 0;
    else break;
  }
  m_SortedEdges = 0;
}
//------------------------------------------------------------------------------

bool Process1Before2(IntersectNode &node1, IntersectNode &node2)
{
  bool result;
  if (node1.pt.Y == node2.pt.Y)
  {
    if (node1.edge1 == node2.edge1 || node1.edge2 == node2.edge1)
    {
      result = node2.pt.X > node1.pt.X;
      if (node2.edge1->dx > 0) return result; else return !result;
    }
    else if (node1.edge1 == node2.edge2 || node1.edge2 == node2.edge2)
    {
      result = node2.pt.X > node1.pt.X;
      if (node2.edge2->dx > 0) return result; else return !result;
    }
    else return node2.pt.X > node1.pt.X;
  }
  else return node1.pt.Y > node2.pt.Y;
}
//------------------------------------------------------------------------------

void Clipper4::AddIntersectNode(TEdge4 *e1, TEdge4 *e2, const IntPoint &pt)
{
  IntersectNode* newNode = new IntersectNode;
  newNode->edge1 = e1;
  newNode->edge2 = e2;
  newNode->pt = pt;
  newNode->next = 0;
  if( !m_IntersectNodes ) m_IntersectNodes = newNode;
  else if(  Process1Before2(*newNode, *m_IntersectNodes) )
  {
    newNode->next = m_IntersectNodes;
    m_IntersectNodes = newNode;
  }
  else
  {
    IntersectNode* iNode = m_IntersectNodes;
    while( iNode->next  && Process1Before2(*iNode->next, *newNode) )
        iNode = iNode->next;
    newNode->next = iNode->next;
    iNode->next = newNode;
  }
}
//------------------------------------------------------------------------------

void Clipper4::ProcessIntersectList()
{
  while( m_IntersectNodes )
  {
    IntersectNode* iNode = m_IntersectNodes->next;
    {
      IntersectEdges( m_IntersectNodes->edge1 ,
        m_IntersectNodes->edge2 , m_IntersectNodes->pt, ipBoth );
      SwapPositionsInAEL( m_IntersectNodes->edge1 , m_IntersectNodes->edge2 );
    }
    delete m_IntersectNodes;
    m_IntersectNodes = iNode;
  }
}
//------------------------------------------------------------------------------

void Clipper4::DoMaxima(TEdge4 *e, int topY)
{
  TEdge4* eMaxPair = GetMaximaPair(e);
  int X = e->xtop;
  TEdge4* eNext = e->nextInAEL;
  while( eNext != eMaxPair )
  {
    if (!eNext) throw clipper4Exception("DoMaxima error");
    IntersectEdges( e, eNext, MakePoint(X, topY), ipBoth );
    eNext = eNext->nextInAEL;
  }
  if( e->outIdx < 0 && eMaxPair->outIdx < 0 )
  {
    DeleteFromAEL( e );
    DeleteFromAEL( eMaxPair );
  }
  else if( e->outIdx >= 0 && eMaxPair->outIdx >= 0 )
  {
    IntersectEdges( e, eMaxPair, MakePoint(X, topY), ipNone );
  }
  else throw clipper4Exception("DoMaxima error");
}
//------------------------------------------------------------------------------

void Clipper4::ProcessEdgesAtTopOfScanbeam(const int topY)
{
  TEdge4* e = m_ActiveEdges;
  while( e )
  {
    //1. process maxima, treating them as if they're 'bent' horizontal edges,
    //   but exclude maxima with horizontal edges. nb: e can't be a horizontal.
    if( IsMaxima(e, topY) && GetMaximaPair(e)->dx != horizontal )
    {
      //'e' might be removed from AEL, as may any following edges so ...
      TEdge4* ePrior = e->prevInAEL;
      DoMaxima(e, topY);
      if( !ePrior ) e = m_ActiveEdges;
      else e = ePrior->nextInAEL;
    }
    else
    {
      //2. promote horizontal edges, otherwise update xcurr and ycurr ...
      if(  IsIntermediate(e, topY) && e->nextInLML->dx == horizontal )
      {
        if (e->outIdx >= 0) AddPolyPt(e, MakePoint(e->xtop, e->ytop));
        UpdateEdgeIntoAEL(e);
        AddEdgeToSEL(e);
      } else
      {
        //this just simplifies horizontal processing ...
        e->xcurr = TopX( *e, topY );
        e->ycurr = topY;
      }
      e = e->nextInAEL;
    }
  }

  //3. Process horizontals at the top of the scanbeam ...
  ProcessHorizontals();

  if (!m_ActiveEdges) return;

  //4. Promote intermediate vertices ...
  e = m_ActiveEdges;
  while( e )
  {
    if( IsIntermediate( e, topY ) )
    {
      if( e->outIdx >= 0 ) AddPolyPt(e, MakePoint(e->xtop,e->ytop));
      UpdateEdgeIntoAEL(e);
    }
    e = e->nextInAEL;
  }
}
//------------------------------------------------------------------------------

PolyPt* FixupOutPolygon(PolyPt *p)
{
  //FixupOutPolygon() - removes duplicate points and simplifies consecutive
  //parallel edges by removing the middle vertex.
  if (!p) return 0;
  PolyPt *pp = p, *result = p, *lastOK = 0;
  for (;;)
  {
    if (pp->prev == pp || pp->prev == pp->next )
    {
      DisposePolyPts(pp);
      return 0;
    }
    //test for duplicate points and for same slope (cross-product) ...
    if ( PointsEqual(pp->pt, pp->next->pt) ||
      SlopesEqual(pp->prev->pt, pp->pt, pp->next->pt) )
    {
      lastOK = 0;
      pp->prev->next = pp->next;
      pp->next->prev = pp->prev;
      PolyPt* tmp = pp;
      if (pp == result) result = pp->prev;
      pp = pp->prev;
      delete tmp;
    }
    else if (pp == lastOK) break;
    else
    {
      if (!lastOK) lastOK = pp;
      pp = pp->next;
    }
  }
  return result;
}
//------------------------------------------------------------------------------

void Clipper4::BuildResult(Polygons &polypoly)
{
  for (PolyPtList::size_type i = 0; i < m_PolyPts.size(); ++i)
    if (m_PolyPts[i])
    {
      m_PolyPts[i] = FixupOutPolygon(m_PolyPts[i]);
      //fix orientation ...
      PolyPt *p = m_PolyPts[i];
      if (p && p->isHole == IsClockwise(p))
        ReversePolyPtLinks(*p);
    }

  int k = 0;
  polypoly.resize(m_PolyPts.size());
  for (unsigned i = 0; i < m_PolyPts.size(); ++i) {
    if (m_PolyPts[i]) {
      Polygon* pg = &polypoly[k];
      pg->clear();
      PolyPt* p = m_PolyPts[i];

      do {
        pg->push_back(p->pt);
        p = p->next;
      } while (p != m_PolyPts[i]);
      if (pg->size() < 3) pg->clear(); else k++;
    }
  }
}
//------------------------------------------------------------------------------

void SwapIntersectNodes(IntersectNode &int1, IntersectNode &int2)
{
  TEdge4 *e1 = int1.edge1;
  TEdge4 *e2 = int1.edge2;
  IntPoint p = int1.pt;

  int1.edge1 = int2.edge1;
  int1.edge2 = int2.edge2;
  int1.pt = int2.pt;

  int2.edge1 = e1;
  int2.edge2 = e2;
  int2.pt = p;
}
//------------------------------------------------------------------------------

bool Clipper4::FixupIntersections()
{
  if ( !m_IntersectNodes->next ) return true;

  CopyAELToSEL();
  IntersectNode *int1 = m_IntersectNodes;
  IntersectNode *int2 = m_IntersectNodes->next;
  while (int2)
  {
    TEdge4 *e1 = int1->edge1;
    TEdge4 *e2;
    if (e1->prevInSEL == int1->edge2) e2 = e1->prevInSEL;
    else if (e1->nextInSEL == int1->edge2) e2 = e1->nextInSEL;
    else
    {
      //The current intersection is out of order, so try and swap it with
      //a subsequent intersection ...
      while (int2)
      {
        if (int2->edge1->nextInSEL == int2->edge2 ||
          int2->edge1->prevInSEL == int2->edge2) break;
        else int2 = int2->next;
      }
      if ( !int2 ) return false; //oops!!!

      //found an intersect node that can be swapped ...
      SwapIntersectNodes(*int1, *int2);
      e1 = int1->edge1;
      e2 = int1->edge2;
    }
    SwapPositionsInSEL(e1, e2);
    int1 = int1->next;
    int2 = int1->next;
  }

  m_SortedEdges = 0;

  //finally, check the last intersection too ...
  return (int1->edge1->prevInSEL == int1->edge2 ||
    int1->edge1->nextInSEL == int1->edge2);
}
//------------------------------------------------------------------------------

bool E2InsertsBeforeE1(TEdge4 &e1, TEdge4 &e2)
{
  if (e2.xcurr == e1.xcurr) return e2.dx > e1.dx;
  else return e2.xcurr < e1.xcurr;
}
//------------------------------------------------------------------------------

void Clipper4::InsertEdgeIntoAEL(TEdge4 *edge)
{
  edge->prevInAEL = 0;
  edge->nextInAEL = 0;
  if( !m_ActiveEdges )
  {
    m_ActiveEdges = edge;
  }
  else if( E2InsertsBeforeE1(*m_ActiveEdges, *edge) )
  {
    edge->nextInAEL = m_ActiveEdges;
    m_ActiveEdges->prevInAEL = edge;
    m_ActiveEdges = edge;
  } else
  {
    TEdge4* e = m_ActiveEdges;
    while( e->nextInAEL  && !E2InsertsBeforeE1(*e->nextInAEL , *edge) )
      e = e->nextInAEL;
    edge->nextInAEL = e->nextInAEL;
    if( e->nextInAEL ) e->nextInAEL->prevInAEL = edge;
    edge->prevInAEL = e;
    e->nextInAEL = edge;
  }
}
//----------------------------------------------------------------------

void Clipper4::DoEdge1(TEdge4 *edge1, TEdge4 *edge2, const IntPoint &pt)
{
  AddPolyPt(edge1, pt);
  SwapSides(*edge1, *edge2);
  SwapPolyIndexes(*edge1, *edge2);
}
//----------------------------------------------------------------------

void Clipper4::DoEdge2(TEdge4 *edge1, TEdge4 *edge2, const IntPoint &pt)
{
  AddPolyPt(edge2, pt);
  SwapSides(*edge1, *edge2);
  SwapPolyIndexes(*edge1, *edge2);
}
//----------------------------------------------------------------------

void Clipper4::DoBothEdges(TEdge4 *edge1, TEdge4 *edge2, const IntPoint &pt)
{
  AddPolyPt(edge1, pt);
  AddPolyPt(edge2, pt);
  SwapSides( *edge1 , *edge2 );
  SwapPolyIndexes( *edge1 , *edge2 );
}
//----------------------------------------------------------------------

bool Clipper4::IsHole(TEdge4 *e)
{
  bool hole = false;
  TEdge4 *e2 = m_ActiveEdges;
  while (e2 && e2 != e)
  {
    if (e2->outIdx >= 0) hole = !hole;
    e2 = e2->nextInAEL;
  }
  return hole;
}
//----------------------------------------------------------------------

} //namespace clipper4

