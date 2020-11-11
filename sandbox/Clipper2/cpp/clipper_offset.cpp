/*******************************************************************************
* Author    :  Angus Johnson                                                   *
* Version   :  10.0 (beta)                                                     *
* Date      :  2 November 2020                                                 *
* Website   :  http://www.angusj.com                                           *
* Copyright :  Angus Johnson 2010-2020                                         *
* Purpose   :  Polygon offsetting                                              *
* License   :  http://www.boost.org/LICENSE_1_0.txt                            *
*                                                                              *
* C++       :  Thanks to help from Andreas Lücke - ALuecke@gmx.net             * 
*******************************************************************************/

#include "clipper_offset.h"
#include "clipper_core.h"
#include "clipper.h"
#include <cmath>

namespace clipperlib {

//------------------------------------------------------------------------------
// Miscellaneous methods
//------------------------------------------------------------------------------

int getLowestPolygonIdx(const PathsD paths)
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

PointD getUnitNormal(const PointD pt1, const PointD pt2)
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

ClipperOffset::ClipperOffset(double miter_limit_, double arc_tolerance_)
{
	miter_limit = miter_limit_;
	arc_tolerance = arc_tolerance_;
	join_type = JoinType::Square;
}

ClipperOffset::~ClipperOffset()
{
	clear();
}

void ClipperOffset::clear()
{
	for (auto gp: group_in)
		delete (gp);
	group_in.clear();
	norms.clear();
}

void ClipperOffset::addPath(const PathD &p, JoinType jt_, EndType et_)
{
	PathsD pp;
	pp.push_back(p);
	addPaths(pp, jt_, et_);
}

void ClipperOffset::addPaths(const PathsD &p, JoinType jt_, EndType et_)
{
	if (p.size() == 0) return;
	PathGroup* pg = new PathGroup(p, jt_, et_);
	group_in.push_back(pg);
}

void ClipperOffset::buildNormals() 
{
	size_t path_size = path_in.size();
	norms.resize(path_size);
	for (size_t j = 0; j < path_size - 1; ++j)
		norms[j] = getUnitNormal(path_in[j], path_in[j + 1]);
	norms[path_size - 1] = getUnitNormal(path_in[path_size - 1], path_in[0]);
}

void ClipperOffset::doOffset(PathGroup& pathGroup, double delta_)
{
	double abs_delta = std::abs(delta);
	double arc_tol;
	double steps;
	PointD norm;
	double x2;
	bool isClockwise = true;

	//CheckPaths();

	if (pathGroup.end_type != EndType::Polygon) delta_ = std::abs(delta_)/2;
	bool isOpenPaths = (pathGroup.end_type != EndType::Polygon &&
		  pathGroup.end_type != EndType::OpenJoined);
	if (!isOpenPaths) {
		int lowest_idx = getLowestPolygonIdx(pathGroup.paths);
		if (lowest_idx < 0) return;

		isClockwise = (pathGroup.paths[lowest_idx].Area() > 0);
		if (!isClockwise) delta_ = -delta_;
	}
	delta = delta_;
	abs_delta = std::abs(delta);
	//miter_limit: see offset_triginometry3.svg
	if(miter_limit > 1.0) 
		miter_lim = 2.0/(miter_limit*miter_limit);
	else 
		miter_lim = 2.0;

	arc_tol = (arc_tolerance <= default_arc_frac ?  default_arc_frac : arc_tol = arc_tolerance);
	join_type = pathGroup.join_type;

	//see offset_triginometry2.svg
	steps = PI / acos(1.0 - arc_tol / abs_delta); //steps per 360 degrees
	//if(steps > abs_delta*PI) steps = abs_delta * PI; //ie excessive precision check
	steps_per_rad = steps / two_pi;

	paths_out.clear();
	for (auto path: pathGroup.paths.data)
	{
		size_t pathSize = path.size();
		path_in = path;
		if (pathSize == 0) continue;
		path_out.clear();

		//if a single vertex then build a circle or a square ...
		if(pathSize == 1)
		{
			if(join_type == JoinType::Round)
			{
				norms.clear();
				norms.push_back(PointD(1, 0));
				norms.push_back(PointD(-1, 0));
				doRound(0, 1, two_pi);
				path_out.pop_back();
			}
			else
			{
				addPoint(path[0].x - delta, path[0].y - delta);
				addPoint(path[0].x + delta, path[0].y - delta);
				addPoint(path[0].x + delta, path[0].y + delta);
				addPoint(path[0].x - delta, path[0].y + delta);
			}
			paths_out.push_back(path_out);
			continue;
		}

		buildNormals();

		if(pathGroup.end_type == EndType::Polygon)
		{
			offsetPolygon();
		}
		else if(pathGroup.end_type == EndType::OpenJoined)		
		{			
			offsetOpenJoined();
		}
		else
		{  
			offsetOpenPath(pathGroup.end_type);
		}
	}

	//clean up 'corners' ...
	Clipper clip;
	PathsI solI;
	solI.AppendPointsScale(paths_out, 100.0);
	clip.AddPaths(solI, PathType::Subject);
	if (isClockwise)
	{
		clip.Execute(ClipType::Union, FillRule::Positive, solI);
	}
	else
	{
		clip.Execute(ClipType::Union, FillRule::Negative, solI);
	}
	paths_out.clear();
	paths_out.AppendPointsScale(solI, 0.01);

	for (auto p : paths_out.data)
		solution.push_back(p);
}

void ClipperOffset::offsetPolygon() {
	size_t pathSize = path_in.size(), k = pathSize - 1;
	for (size_t j = 0; j < pathSize; ++j)
		offsetPoint(j, k);
	paths_out.push_back(path_out);
}

void ClipperOffset::offsetOpenJoined() {
	offsetPolygon();
	path_out.clear();
	path_in.Reverse();
	buildNormals();
	offsetPolygon();
}

void ClipperOffset::offsetOpenPath(EndType et) {
	
	size_t pathSize = path_in.size(), k = 0;
	for (int j = 1; j < pathSize - 1; ++j)
		offsetPoint(j, k);

	size_t j = pathSize - 1;
	k = j - 1;
	norms[pathSize - 1] = -norms[k];

	//handle the end (butt, round or square) ...
	if (et == EndType::OpenButt)
	{
		addPoint(path_in[j] + norms[k] * delta);
		addPoint(path_in[j] - norms[k] * delta);
	}
	else if (et == EndType::OpenSquare)
	{
		doSquare(j, k);
	}
	else
	{
		doRound(j, k, PI);
	}

	//reverse normals ...
	for (int i = pathSize - 1; i > 0; --i)
		norms[i] = -norms[i - 1];
	norms[0] = -norms[1];


	//repeat offset but now going backward ...
	k = pathSize - 1;
	for (int j = k - 1; j > 0; --j) offsetPoint(j, k);

	//finally handle the start (butt, round or square) ...
	if (et == EndType::OpenButt)
	{
		addPoint(path_in[0] + norms[1] * delta);
		addPoint(path_in[0] - norms[1] * delta);
	}
	else if (et == EndType::OpenSquare)
	{
		doSquare(0, 1);
	}
	else
	{
		doRound(0, 1, PI);
	}
	paths_out.push_back(path_out);
}

PathsD ClipperOffset::execute(double delta_)
{
	solution.clear();
	if (group_in.size() == 0) return solution;

	//if a Zero offset, then just copy CLOSED polygons to FSolution and return ...
	if (std::abs(delta_) < tolerance)
	{
		size_t sol_count = 0;
		for (size_t i = 0; i < group_in.size(); i++)
		{
			if (group_in[i]->end_type == EndType::Polygon)
			{
				for (auto p : group_in[i]->paths.data)
					solution.push_back(p);
			}
		}
		return solution;
	}

	//nb: delta will depend on whether paths are polygons or open
	for (size_t i = 0; i < group_in.size(); i++) {
		doOffset(*group_in[i], delta_);
	}
	return solution;
}

void ClipperOffset::doSquare(size_t j, size_t k)
{
	//Two vertices, one using the prior offset's (k) normal one the current (j).
	//Do a 'normal' offset (by delta) and then another by 'de-normaling' the
     //normal hence parallel to the direction of the respective edges.
	if(delta > 0.0)
	{
		addPoint(path_in[j].x + delta*(norms[k].x-norms[k].y),
						path_in[j].y + delta*(norms[k].y+norms[k].x));
		addPoint(path_in[j].x + delta*(norms[j].x+norms[j].y),
						path_in[j].y + delta*(norms[j].y-norms[j].x));
	}
	else
	{
		addPoint(path_in[j].x + delta*(norms[k].x+norms[k].y),
					    path_in[j].y + delta*(norms[k].y-norms[k].x));
		addPoint(path_in[j].x + delta*(norms[j].x-norms[j].y),
						path_in[j].y + delta*(norms[j].y+norms[j].x));
	}
}

void ClipperOffset::doMiter(size_t j, size_t k, double cosAplus1)
{
	//see offset_triginometry4.svg
	double q = delta/cosAplus1;  //0 < cosAplus1 <= 2
	addPoint(path_in[j]+(norms[k]+norms[j])*q);
}

void ClipperOffset::doRound(size_t j, size_t k, double angle)
{
	//even though angle may be negative this is a convex join
	PointD p = norms[k] * delta;
	PointD q = path_in[j];
	addPoint(q + p);

	int steps = static_cast<int>(steps_per_rad * std::abs(angle));
	if (steps > 0) {
		double stepSin = sin(angle / steps);
		double stepCos = cos(angle / steps);

		for (int i = 1; i < steps; i++)
		{
			double x2 = p.x;
			p.x = p.x * stepCos - stepSin * p.y;
			p.y = x2 * stepSin + p.y * stepCos;
			addPoint(q + p);
		}
	}

	p = norms[j] * delta;
	q = path_in[j];
	addPoint(q + p);
}

void ClipperOffset::offsetPoint(size_t j, size_t &k)
{
	//A: angle between adjoining paths on left side (left WRT winding direction).
	//A == 0 deg (or A == 360 deg): collinear edges heading in same direction
	//A == 180 deg: collinear edges heading in opposite directions (ie a 'spike')
	//sin(A) < 0: convex on left.
	//cos(A) > 0: angles on both left and right sides > 90 degrees

	//cross product ...
	sin_val = norms[k].x*norms[j].y - norms[j].x*norms[k].y;
	if (sin_val < 0.001 && sin_val > -0.001) { k = j; return; }
	else if(sin_val > 1.0) sin_val = 1.0;
	else if(sin_val < -1.0) sin_val = -1.0;

	if(sin_val * delta < 0.0) //ie a concave offset
	 {
		 addPoint(path_in[j]+norms[k]*delta);
		 addPoint(path_in[j]); //this improves clipping removal later
		 addPoint(path_in[j]+norms[j]*delta);
	 }
	 else
	 {
		//convex offsets here ...
		 switch(join_type)
		 {
		 case JoinType::Miter:
			 //see offset_triginometry3.svg
			 if((1.0 + cos_val)<miter_lim) doSquare(j, k);
			 else doMiter(j, k, 1.0 + cos_val);
			 break;
		 case JoinType::Square:
			 cos_val = norms[k].x * norms[j].x + norms[j].y * norms[k].y;
			 if(cos_val >= 0.0) //angles >= 90 deg. don't need squaring
			 {
				 doMiter(j, k, 1+cos_val);
			 }
			 else
			 {
				 doSquare(j, k);
			 }
			 break;
		 case JoinType::Round:
			 cos_val = norms[k].x * norms[j].x + norms[j].y * norms[k].y;
			 doRound(j, k, atan2(sin_val, cos_val));
			 break;
		 }
	 }
	 k=j;
}

PathsI clipperOffsetPaths(const PathsI paths, double delta, JoinType jt, EndType et)
{
	ClipperOffset clip_offset;
	clip_offset.addPaths(paths, jt, et);
	return clip_offset.execute(delta);
}

PathsD clipperOffsetPaths(const PathsD paths, double delta, JoinType jt, EndType et)
{
	ClipperOffset clip_offset;
	clip_offset.addPaths(paths, jt, et);
	return clip_offset.execute(delta);
}


}
