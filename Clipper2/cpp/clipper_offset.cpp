/*******************************************************************************
* Author    :  Angus Johnson                                                   *
* Version   :  10.0 (beta)                                                     *
* Date      :  21 November 2020                                                *
* Website   :  http://www.angusj.com                                           *
* Copyright :  Angus Johnson 2010-2020                                         *
* Purpose   :  Polygon offsetting                                              *
* License   :  http://www.boost.org/LICENSE_1_0.txt                            *
*                                                                              *
* C++       :  Thanks to help from Andreas Lücke - ALuecke@gmx.net             * 
*******************************************************************************/

#include "clipper_offset.h"
#include "clipper.h"
#include <cmath>
#include "clipper_core.hpp"

namespace clipperlib {

//------------------------------------------------------------------------------
// Miscellaneous methods
//------------------------------------------------------------------------------

int GetLowestPolygonIdx(const PathsD &paths)
{
	int lp_idx = -1;
	PointD lp;
	for (int i = 0; i < static_cast<int>(paths.size()); ++i)
		if (paths[i].size() > 0) {
			lp_idx = i;
			lp = paths[i][0];
			break;
		}
	if (lp_idx < 0) return lp_idx;

	for (int i = lp_idx; i < static_cast<int>(paths.size()); ++i)
	{
		PathD p = paths[i];
		for (size_t j = 0; j < p.size(); j++) {
			if (p[j].y > lp.y || (p[j].y == lp.y && p[j].x < lp.x)) {
				lp_idx = i;
				lp = p[j];
			}
		}
	}
	return lp_idx;
}

PointD GetUnitNormal(const PointD pt1, const PointD pt2)
{
	double dx, dy, inverse_hypot;
	if (pt1 == pt2) return PointD(0.0, 0.0);

	dx = pt2.x - pt1.x;
	dy = pt2.y - pt1.y;
	inverse_hypot = 1.0 / hypot(dx, dy);
	dx *= inverse_hypot;
	dy *= inverse_hypot;
	return PointD(dy, -dx);
}

//------------------------------------------------------------------------------
// ClipperOffset methods
//------------------------------------------------------------------------------

void ClipperOffset::Clear()
{
	for (auto gp: group_in)
		delete (gp);
	group_in.clear();
	norms.clear();
}

void ClipperOffset::AddPath(const PathD &p, JoinType jt_, EndType et_)
{
	PathsD pp;
	pp.push_back(p);
	AddPaths(pp, jt_, et_);
}

void ClipperOffset::AddPaths(const PathsD &p, JoinType jt_, EndType et_)
{
	if (p.size() == 0) return;
	PathGroup* pg = new PathGroup(p, jt_, et_);
	group_in.push_back(pg);
}

void ClipperOffset::BuildNormals() 
{
	size_t path_size = path_in.size();
	norms.resize(path_size);
	for (size_t j = 0; j < path_size - 1; ++j)
		norms[j] = GetUnitNormal(path_in[j], path_in[j + 1]);
	norms[path_size - 1] = GetUnitNormal(path_in[path_size - 1], path_in[0]);
}

void ClipperOffset::DoOffset(PathGroup& pathGroup, double delta_)
{
	double steps;
	PointD norm;
	bool isClockwise = true;

	if (pathGroup.end_type != EndType::Polygon) delta_ = std::abs(delta_)/2;
	if (pathGroup.end_type == EndType::Polygon || pathGroup.end_type == EndType::Joined) {
		int lowest_idx = GetLowestPolygonIdx(pathGroup.paths);
		if (lowest_idx < 0) return;
		isClockwise = (pathGroup.paths[lowest_idx].Area() > 0);
		if (!isClockwise) delta_ = -delta_;
	}

	join_type = pathGroup.join_type;
	delta = delta_;
	double arc_tol, abs_delta = std::abs(delta);

	//arc tolerance: see offset_triginometry2.svg
	if (arc_tolerance > 0)
		arc_tol = arc_tolerance;
	else
		arc_tol = std::log10(2 + abs_delta) * 0.25; //empirically derived

	steps = PI / acos(1.0 - arc_tol / abs_delta); //steps per 360 degrees
	//if(steps > abs_delta*PI) steps = abs_delta * PI; //ie excessive precision check
	steps_per_rad = steps / two_pi;

	paths_out.clear();
	
	for (std::vector<Path<double> >::const_iterator it = pathGroup.paths.data.cbegin();
		it != pathGroup.paths.data.cend(); ++ it)
	{
		path_out.clear();
		path_in = *it;

		//make sure the path's edges aren't too short
		bool is_closed_path = pathGroup.end_type == EndType::Polygon;
		path_in.StripDuplicates(is_closed_path, min_edge_len);

		size_t pathSize = path_in.size();
		if (pathSize == 0 || (pathGroup.end_type == EndType::Polygon && pathSize < 3)) continue;

		//if a single vertex then build a circle or a square ...
		if(pathSize == 1)
		{
			isClockwise = true;
			if(join_type == JoinType::Round)
			{
				norms.clear();
				norms.push_back(PointD(1, 0));
				norms.push_back(PointD(-1, 0));
				DoRound(0, 1, two_pi);
				path_out.pop_back();
			}
			else
			{
				AddPoint(path_in[0].x - delta, path_in[0].y - delta);
				AddPoint(path_in[0].x + delta, path_in[0].y - delta);
				AddPoint(path_in[0].x + delta, path_in[0].y + delta);
				AddPoint(path_in[0].x - delta, path_in[0].y + delta);
			}
			paths_out.push_back(path_out);
			continue;
		}

		BuildNormals();

		if(pathGroup.end_type == EndType::Polygon)
		{
			OffsetPolygon();
		}
		else if(pathGroup.end_type == EndType::Joined)		
		{			
			OffsetOutline();
		}
		else
		{  
			OffsetOpenLine(pathGroup.end_type);
		}
	}

	if (!isClockwise)
		paths_out.Reverse();

	if (!merge_groups) {
		//clean up 'corners' ...
		ClipperD clipper;
		clipper.AddPaths(paths_out, PathType::Subject);
		clipper.Execute(ClipType::Union, FillRule::Positive, paths_out);
	}

	for (std::vector<Path<double> >::const_iterator it = paths_out.data.cbegin();
		it != paths_out.data.cend(); ++it) solution.push_back(*it);
}

void ClipperOffset::OffsetPolygon() {
	size_t pathSize = path_in.size(), k = pathSize - 1;
	for (size_t j = 0; j < pathSize; ++j)
		OffsetPoint(j, k);
	paths_out.push_back(path_out);
}

void ClipperOffset::OffsetOutline() {
	OffsetPolygon();
	path_out.clear();
	path_in.Reverse();
	BuildNormals();
	OffsetPolygon();
}

void ClipperOffset::OffsetOpenLine(EndType et) {
	
	size_t pathSize = path_in.size(), k = 0;
	if (pathSize < 2) return;

	for (size_t j = 1; j < pathSize - 1; ++j)
		OffsetPoint(j, k);

	size_t j = pathSize - 1;
	k = j - 1;
	norms[pathSize - 1] = -norms[k];

	//handle the end (butt, round or square) ...
	if (et == EndType::Butt)
	{
		AddPoint(path_in[j] + norms[k] * delta);
		AddPoint(path_in[j] - norms[k] * delta);
	}
	else if (et == EndType::Square)
	{
		DoSquare(j, k);
	}
	else
	{
		DoRound(j, k, PI);
	}

	//reverse normals ...
	for (size_t i = pathSize - 1; i > 0; --i)
		norms[i] = -norms[i - 1];
	norms[0] = -norms[1];


	//repeat offset but now going backward ...
	k = pathSize - 1;
	for (size_t l = k - 1; l > 0; --l) OffsetPoint(l, k);

	//finally handle the start (butt, round or square) ...
	if (et == EndType::Butt)
	{
		AddPoint(path_in[0] + norms[1] * delta);
		AddPoint(path_in[0] - norms[1] * delta);
	}
	else if (et == EndType::Square)
	{
		DoSquare(0, 1);
	}
	else
	{
		DoRound(0, 1, PI);
	}
	paths_out.push_back(path_out);
}

PathsD ClipperOffset::Execute(double delta_)
{
	solution.clear();
	if (group_in.size() == 0) return solution;

	//if a Zero offset, then just copy CLOSED polygons to FSolution and return ...
	if (std::abs(delta_) < floating_point_tolerance)
	{
		for (size_t i = 0; i < group_in.size(); i++)
		{
			if (group_in[i]->end_type == EndType::Polygon)
			{
				for (std::vector<Path<double> >::const_iterator 
					it = group_in[i]->paths.data.cbegin(); 
					it != group_in[i]->paths.data.cend(); 
					++it) solution.push_back(*it);
			}
		}
		return solution;
	}

	//miter_limit: see offset_triginometry3.svg
	if (miter_limit > 1.0)
		miter_lim = 2.0 / (miter_limit * miter_limit);
	else
		miter_lim = 2.0;

	if (min_edge_len < floating_point_tolerance) 
		min_edge_len = default_min_edge_len;
	min_edge_len_sqrd = min_edge_len * min_edge_len;

	//nb: delta will depend on whether paths are polygons or open
	for (size_t i = 0; i < group_in.size(); i++) {
		DoOffset(*group_in[i], delta_);
	}

	if (merge_groups) {
		//clean up 'corners' ...
		ClipperD clipper;
		clipper.AddPaths(solution, PathType::Subject);
		clipper.Execute(ClipType::Union, FillRule::Positive, solution);
	}
	return solution;
}

void ClipperOffset::DoSquare(size_t j, size_t k)
{
	//Two vertices, one using the prior offset's (k) normal one the current (j).
	//Do a 'normal' offset (by delta) and then another by 'de-normaling' the
     //normal hence parallel to the direction of the respective edges.
	if(delta > 0.0)
	{
		AddPoint(path_in[j].x + delta*(norms[k].x-norms[k].y),
						path_in[j].y + delta*(norms[k].y+norms[k].x));
		AddPoint(path_in[j].x + delta*(norms[j].x+norms[j].y),
						path_in[j].y + delta*(norms[j].y-norms[j].x));
	}
	else
	{
		AddPoint(path_in[j].x + delta*(norms[k].x+norms[k].y),
					    path_in[j].y + delta*(norms[k].y-norms[k].x));
		AddPoint(path_in[j].x + delta*(norms[j].x-norms[j].y),
						path_in[j].y + delta*(norms[j].y+norms[j].x));
	}
}

void ClipperOffset::DoMiter(size_t j, size_t k, double cosAplus1)
{
	//see offset_triginometry4.svg
	double q = delta/cosAplus1;  //0 < cosAplus1 <= 2
	AddPoint(path_in[j]+(norms[k]+norms[j])*q);
}

void ClipperOffset::DoRound(size_t j, size_t k, double angle)
{
	//even though angle may be negative this is a convex join
	PointD p = norms[k] * delta;
	PointD q = path_in[j];
	AddPoint(q + p);

	int steps = static_cast<int>(ceil(steps_per_rad * std::abs(angle)));
	if (steps > 0) {
		double stepSin = sin(angle / steps);
		double stepCos = cos(angle / steps);

		for (int i = 1; i < steps; i++)
		{
			double x2 = p.x;
			p.x = p.x * stepCos - stepSin * p.y;
			p.y = x2 * stepSin + p.y * stepCos;
			AddPoint(q + p);
		}
	}
	AddPoint(path_in[j] + norms[j] * delta);
}

void ClipperOffset::OffsetPoint(size_t j, size_t &k)
{
	//A: angle between adjoining paths on left side (left WRT winding direction).
	//A == 0 deg (or A == 360 deg): collinear edges heading in same direction
	//A == 180 deg: collinear edges heading in opposite directions (ie a 'spike')
	//sin(A) < 0: convex on left.
	//cos(A) > 0: angles on both left and right sides > 90 degrees

	//cross product ...
	sin_val = norms[k].x*norms[j].y - norms[j].x*norms[k].y;
	//if (sin_val < 0.005 && sin_val > -0.005) return; //ie very near colinear
	if(sin_val > 1.0) sin_val = 1.0;
	else if(sin_val < -1.0) sin_val = -1.0;

	if(sin_val * delta < 0.0) //ie a concave offset
	 {
		PointD p1 = path_in[j] + norms[k] * delta;
		PointD p2 = path_in[j] + norms[j] * delta;
		AddPoint(p1);
		if (!NearEqual(p1, p2, min_edge_len_sqrd)) {
			AddPoint(path_in[j]); //nb: adding this point aids clipping removal later
			AddPoint(p2);
		}
	 }
	 else
	 {
		//convex offsets here ...
		cos_val = norms[k].x * norms[j].x + norms[j].y * norms[k].y;
		switch(join_type)
		 {
		 case JoinType::Miter:
			 //see offset_triginometry3.svg
			 if((1.0 + cos_val)<miter_lim) DoSquare(j, k);
			 else DoMiter(j, k, 1.0 + cos_val);
			 break;
		 case JoinType::Square:
			 //angles < 90 deg. should be squared
			 if(cos_val < 0.0) 
			 {
				 DoSquare(j, k);
			 }
			 else
			 {
				 DoMiter(j, k, 1 + cos_val); 
			 }
			 break;
		 case JoinType::Round:
			 DoRound(j, k, atan2(sin_val, cos_val));
			 break;
		 }
	 }
	 k=j;
}

PathsI InflatePaths(const PathsI &paths, double delta, JoinType jt, EndType et)
{
	ClipperOffset clip_offset;
	clip_offset.AddPaths(paths, jt, et);
	return clip_offset.Execute(delta);
}

PathsD InflatePaths(const PathsD &paths, double delta, JoinType jt, EndType et)
{
	ClipperOffset clip_offset;
	clip_offset.AddPaths(paths, jt, et);
	return clip_offset.Execute(delta);
}


}
