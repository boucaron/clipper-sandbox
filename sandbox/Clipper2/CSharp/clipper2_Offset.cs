﻿/*******************************************************************************
*                                                                              *
* Author    :  Angus Johnson                                                   *
* Version   :  10.0 (alpha)                                                    *
* Date      :  9 September 2017                                                *
* Website   :  http://www.angusj.com                                           *
* Copyright :  Angus Johnson 2010-2017                                         *
*                                                                              *
* License:                                                                     *
* Use, modification & distribution is subject to Boost Software License Ver 1. *
* http://www.boost.org/LICENSE_1_0.txt                                         *
*                                                                              *
*******************************************************************************/

using System;
using System.Collections.Generic;

namespace ClipperLib
{

  using Path = List<Point64>;
  using Paths = List<List<Point64>>;

  public enum JoinType { Square, Round, Miter };
  public enum EndType { Polygon, OpenJoined, OpenButt, OpenSquare, OpenRound };

  public struct PointD
  {
    public double X;
    public double Y;

    public PointD(double x = 0, double y = 0)
    {
      this.X = x; this.Y = y;
    }
    public PointD(PointD dp)
    {
      this.X = dp.X; this.Y = dp.Y;
    }
    public PointD(Point64 ip)
    {
      this.X = ip.X; this.Y = ip.Y;
    }
  };

  internal class PathNode
  {
    internal Path path;
    internal JoinType joinType;
    internal EndType endType;
    internal Int32 lowestIdx;

    public PathNode(Path p, JoinType jt, EndType et)
    {
      path = p;
      joinType = jt;
      endType = et;

      Int32 lenP = p.Count;
      if (et == EndType.Polygon || et == EndType.OpenJoined)
        while (lenP > 1 && p[lenP - 1] == p[0]) lenP--;
      else if (lenP == 2 && p[1] == p[0])
        lenP = 1;
      if (lenP == 0) return;

      if (lenP < 3 && (et == EndType.Polygon || et == EndType.OpenJoined))
      {
        if (jt == JoinType.Round) endType = EndType.OpenRound;
        else endType = EndType.OpenSquare;
      }

      path = new Path(lenP);
      path.Add(p[0]);

      Point64 lastIp = p[0];
      lowestIdx = 0;
      for (Int32 i = 1; i < lenP; i++)
      {
        if (lastIp == p[i]) continue;
        path.Add(p[i]);
        lastIp = p[i];
        if (et != EndType.Polygon) continue;
        if (p[i].Y >= path[lowestIdx].Y &&
          (p[i].Y > path[lowestIdx].Y || p[i].X < path[lowestIdx].X))
          lowestIdx = i;
      }
      if (endType == EndType.Polygon && path.Count < 3) path = null;
    }
  } //PathNode class


  public class Clipper2Offset
  {
    private double delta, sinA, sin, cos;
    //nb: miterLim below is a temp field that differs from the MiterLimit property
    private double miterLim, stepsPerRad; 
    private Paths solution;
    private Path pathIn, pathOut;
    private List<PointD> norms = new List<PointD>();
    private List<PathNode> nodes = new List<PathNode>();
    private Int32 lowestIdx;
    public double ArcTolerance { get; set; }
    public double MiterLimit { get; set; }
    private Point64 PointZero = new Point64(0, 0);

    private const double TwoPi = Math.PI * 2;
    private const double DefaultArcTolerance = 0.2;
    private const double Tolerance = 1.0E-15;

    //------------------------------------------------------------------------------

    internal static Int64 Round(double value)
    {
      return value < 0 ? (Int64)(value - 0.5) : (Int64)(value + 0.5);
    }
    //------------------------------------------------------------------------------

    public static double Area(Path p)
    {
      int cnt = (int)p.Count;
      if (cnt < 3) return 0;
      double a = 0;
      for (int i = 0, j = cnt - 1; i < cnt; ++i)
      {
        a += ((double)p[j].X + p[i].X) * ((double)p[j].Y - p[i].Y);
        j = i;
      }
      return -a * 0.5;
    }
    //------------------------------------------------------------------------------

    public Clipper2Offset() { ArcTolerance = DefaultArcTolerance; MiterLimit = 2.0; }
    //------------------------------------------------------------------------------

    public void Clear()  { nodes.Clear(); }
    //------------------------------------------------------------------------------

    public void AddPath(Path p, JoinType jt, EndType et)
    {
      PathNode pn = new PathNode(p, jt, et);
      if (pn.path == null) pn = null;
      else nodes.Add(pn);
    }
    //------------------------------------------------------------------------------

    public void AddPaths(Paths paths, JoinType jt, EndType et)
    {
      foreach (Path p in paths) AddPath(p, jt, et);
    }
    //------------------------------------------------------------------------------

    private void GetLowestPolygonIdx()
    {
      lowestIdx = -1;
      Point64 ip1 = PointZero, ip2;
      for (Int32 i = 0; i < nodes.Count; i++)
      {
        PathNode node = nodes[i];
        if (node.endType != EndType.Polygon) continue;
        if (lowestIdx < 0)
        {
          ip1 = node.path[node.lowestIdx];
          lowestIdx = i;
        }
        else
        {
          ip2 = node.path[node.lowestIdx];
          if (ip2.Y >= ip1.Y && (ip2.Y > ip1.Y || ip2.X < ip1.X))
          {
            lowestIdx = i;
            ip1 = ip2;
          }
        }
      }
    }
    //------------------------------------------------------------------------------

    internal static PointD GetUnitNormal(Point64 pt1, Point64 pt2)
    {
      double dx = (pt2.X - pt1.X);
      double dy = (pt2.Y - pt1.Y);
      if ((dx == 0) && (dy == 0)) return new PointD();

      double f = 1 * 1.0 / Math.Sqrt(dx * dx + dy * dy);
      dx *= f;
      dy *= f;

      return new PointD(dy, -dx);
    }
    //------------------------------------------------------------------------------

    void OffsetPoint(int j, ref int k, JoinType jointype)
    {
      //cross product ...
      sinA = (norms[k].X * norms[j].Y - norms[j].X * norms[k].Y);

      if (Math.Abs(sinA * delta) < 1.0) //angle is approaching 180 or 360 deg.
      {
        //dot product ...
        double cosA = (norms[k].X * norms[j].X + norms[j].Y * norms[k].Y);
        if (cosA > 0) //given condition above the angle is approaching 360 deg.
        {
          //with angles approaching 360 deg. collinear (whether concave or convex),
          //offsetting with two or more vertices (that are so close together) can
          //occasionally cause tiny self-intersections due to rounding.
          //So we offset with just a single vertex here ...
          pathOut.Add(new Point64(Round(pathIn[j].X + norms[k].X * delta),
            Round(pathIn[j].Y + norms[k].Y * delta)));
          return;
        }
      }
      else if (sinA > 1.0) sinA = 1.0;
      else if (sinA < -1.0) sinA = -1.0;

      if (sinA * delta < 0) //ie a concave offset
      {
        pathOut.Add(new Point64(Round(pathIn[j].X + norms[k].X * delta),
          Round(pathIn[j].Y + norms[k].Y * delta)));
        pathOut.Add(pathIn[j]);
        pathOut.Add(new Point64(Round(pathIn[j].X + norms[j].X * delta),
          Round(pathIn[j].Y + norms[j].Y * delta)));
      }
      else
      {
        //convex offsets here ...
        switch (jointype)
        {
          case JoinType.Miter:
            double cosA = (norms[j].X * norms[k].X + norms[j].Y * norms[k].Y);
            //see offset_triginometry3.svg
            if (1 + cosA < miterLim) DoSquare(j, k);
            else DoMiter(j, k, 1 + cosA);
            break;
          case JoinType.Square:
            cosA = (norms[j].X * norms[k].X + norms[j].Y * norms[k].Y);
            if (cosA >= 0) DoMiter(j, k, 1 + cosA); //angles >= 90 deg. don't need squaring
            else DoSquare(j, k);
            break;
          case JoinType.Round:
            DoRound(j, k);
            break;
        }
      }
      k = j;
    }
    //------------------------------------------------------------------------------

    internal void DoSquare(int j, int k)
    {
      double dx = Math.Tan(Math.Atan2(sinA,
        norms[k].X * norms[j].X + norms[k].Y * norms[j].Y) / 4);
      pathOut.Add(new Point64(
        Round(pathIn[j].X + delta * (norms[k].X - norms[k].Y * dx)),
        Round(pathIn[j].Y + delta * (norms[k].Y + norms[k].X * dx))));
      pathOut.Add(new Point64(
        Round(pathIn[j].X + delta * (norms[j].X + norms[j].Y * dx)),
        Round(pathIn[j].Y + delta * (norms[j].Y - norms[j].X * dx))));
    }
    //------------------------------------------------------------------------------

    internal void DoMiter(int j, int k, double r)
    {
      double q = delta / r;
      pathOut.Add(new Point64(Round(pathIn[j].X + (norms[k].X + norms[j].X) * q),
        Round(pathIn[j].Y + (norms[k].Y + norms[j].Y) * q)));
    }
    //------------------------------------------------------------------------------

    internal void DoRound(int j, int k)
    {
      double a = Math.Atan2(sinA,
      norms[k].X * norms[j].X + norms[k].Y * norms[j].Y);
      int steps = Math.Max((int)Round(stepsPerRad * Math.Abs(a)), 1);

      double X = norms[k].X, Y = norms[k].Y, X2;
      for (int i = 0; i < steps; ++i)
      {
        pathOut.Add(new Point64(
          Round(pathIn[j].X + X * delta),
          Round(pathIn[j].Y + Y * delta)));
        X2 = X;
        X = X * cos - sin * Y;
        Y = X2 * sin + Y * cos;
      }
      pathOut.Add(new Point64(
      Round(pathIn[j].X + norms[j].X * delta),
      Round(pathIn[j].Y + norms[j].Y * delta)));
    }
    //------------------------------------------------------------------------------
    private void DoOffset(double d)
    {
      solution = null;
      delta = d;
      double absDelta = Math.Abs(d);

      //if a Zero offset, then just copy CLOSED polygons to FSolution and return ...
      if (absDelta < Tolerance)
      {
        solution = new Paths(nodes.Count);
        foreach (PathNode node in nodes)
          if (node.endType == EndType.Polygon) solution.Add(node.path);
        return;
      }

      //MiterLimit: see offset_triginometry3.svg in the documentation folder ...
      if (MiterLimit > 2)
        miterLim = 2 / (MiterLimit * MiterLimit);
      else
        miterLim = 0.5;

      double arcTol;
      if (ArcTolerance <= Tolerance) arcTol = DefaultArcTolerance;
      else if (ArcTolerance > absDelta * DefaultArcTolerance) arcTol = absDelta * DefaultArcTolerance;
      else arcTol = ArcTolerance;

      //see offset_triginometry2.svg in the documentation folder ...
      double steps = Math.PI / Math.Acos(1 - arcTol / absDelta);  //steps per 360 degrees
      if (steps > absDelta * Math.PI) steps = absDelta * Math.PI; //ie excessive precision check

      sin = Math.Sin(TwoPi / steps);
      cos = Math.Cos(TwoPi / steps);
      if (d < 0) sin = -sin;
      stepsPerRad = steps / TwoPi;

      solution = new Paths(nodes.Count * 2);
      foreach (PathNode node in nodes)
      {
        pathIn = node.path;
        pathOut = new Path();
        Int32 pathInCnt = pathIn.Count;

        //if a single vertex then build circle or a square ...
        if (pathInCnt == 1)
        {
          if (node.joinType == JoinType.Round)
          {
            double X = 1.0, Y = 0.0;
            for (int j = 1; j <= steps; j++)
            {
              pathOut.Add(new Point64(
                Round(pathIn[0].X + X * delta),
                Round(pathIn[0].Y + Y * delta)));
              double X2 = X;
              X = X * cos - sin * Y;
              Y = X2 * sin + Y * cos;
            }
          }
          else
          {
            double X = -1.0, Y = -1.0;
            for (int j = 0; j < 4; ++j)
            {
              pathOut.Add(new Point64(
                Round(pathIn[0].X + X * delta),
                Round(pathIn[0].Y + Y * delta)));
              if (X < 0) X = 1;
              else if (Y < 0) Y = 1;
              else X = -1;
            }
          }
          solution.Add(pathOut);
          continue;
        } //end of single vertex offsetting

        //build norms ...
        norms.Clear();
        norms.Capacity = pathInCnt;
        for (int j = 0; j < pathInCnt - 1; j++)
          norms.Add(GetUnitNormal(pathIn[j], pathIn[j + 1]));
        if (node.endType == EndType.OpenJoined || node.endType == EndType.Polygon)
          norms.Add(GetUnitNormal(pathIn[pathInCnt - 1], pathIn[0]));
        else
          norms.Add(new PointD(norms[pathInCnt - 2]));

        if (node.endType == EndType.Polygon)
        {
          int k = pathInCnt - 1;
          for (int j = 0; j < pathInCnt; j++)
            OffsetPoint(j, ref k, node.joinType);
          solution.Add(pathOut);
        }
        else if (node.endType == EndType.OpenJoined)
        {
          int k = pathInCnt - 1;
          for (int j = 0; j < pathInCnt; j++)
            OffsetPoint(j, ref k, node.joinType);
          solution.Add(pathOut);
          pathOut = new Path();
          //re-build norms ...
          PointD n = norms[pathInCnt - 1];
          for (int j = pathInCnt - 1; j > 0; j--)
            norms[j] = new PointD(-norms[j - 1].X, -norms[j - 1].Y);
          norms[0] = new PointD(-n.X, -n.Y);
          k = 0;
          for (int j = pathInCnt - 1; j >= 0; j--)
            OffsetPoint(j, ref k, node.joinType);
          solution.Add(pathOut);
        }
        else
        {
          int k = 0;
          for (int j = 1; j < pathInCnt - 1; j++)
            OffsetPoint(j, ref k, node.joinType);

          Point64 pt1;
          if (node.endType == EndType.OpenButt)
          {
            int j = pathInCnt - 1;
            pt1 = new Point64((Int64)Round(pathIn[j].X + norms[j].X *
              delta), (Int64)Round(pathIn[j].Y + norms[j].Y * delta));
            pathOut.Add(pt1);
            pt1 = new Point64((Int64)Round(pathIn[j].X - norms[j].X *
              delta), (Int64)Round(pathIn[j].Y - norms[j].Y * delta));
            pathOut.Add(pt1);
          }
          else
          {
            int j = pathInCnt - 1;
            k = pathInCnt - 2;
            sinA = 0;
            norms[j] = new PointD(-norms[j].X, -norms[j].Y);
            if (node.endType == EndType.OpenSquare)
              DoSquare(j, k);
            else
              DoRound(j, k);
          }

          //reverse norms ...
          for (int j = pathInCnt - 1; j > 0; j--)
            norms[j] = new PointD(-norms[j - 1].X, -norms[j - 1].Y);
          norms[0] = new PointD(-norms[1].X, -norms[1].Y);

          k = pathInCnt - 1;
          for (int j = k - 1; j > 0; --j) OffsetPoint(j, ref k, node.joinType);

          if (node.endType == EndType.OpenButt)
          {
            pt1 = new Point64((Int64)Round(pathIn[0].X - norms[0].X * delta),
              (Int64)Round(pathIn[0].Y - norms[0].Y * delta));
            pathOut.Add(pt1);
            pt1 = new Point64((Int64)Round(pathIn[0].X + norms[0].X * delta),
              (Int64)Round(pathIn[0].Y + norms[0].Y * delta));
            pathOut.Add(pt1);
          }
          else
          {
            k = 1;
            sinA = 0;
            if (node.endType == EndType.OpenSquare)
              DoSquare(0, 1);
            else
              DoRound(0, 1);
          }
          solution.Add(pathOut);
        }
      }
    }
    //------------------------------------------------------------------------------

    public void Execute(ref Paths sol, double delta)
    {
      sol.Clear();
      if (nodes.Count == 0) return;

      GetLowestPolygonIdx();
      if (lowestIdx >= 0 && Area(nodes[lowestIdx].path) < 0) //todo don't use Area :)
      delta = -delta; 

      DoOffset(delta);

      //now clean up 'corners' ...
      Clipper2 clpr = new Clipper2();
      clpr.AddPaths(solution, PolyType.Subject);
      clpr.Execute(ClipType.Union, sol, FillType.Positive);
    }
    //------------------------------------------------------------------------------

    public static Paths OffsetPaths(Paths pp, double delta, JoinType jt, EndType et)
    {
      Paths result = new Paths();
      Clipper2Offset co = new Clipper2Offset();
      co.AddPaths(pp, jt, et);
      co.Execute(ref result, delta);
      return result;
    }
//------------------------------------------------------------------------------

  } //Clipper2Offset
} //namespace
