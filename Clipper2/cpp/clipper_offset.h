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

#ifndef CLIPPER_OFFSET_H_
#define CLIPPER_OFFSET_H_

#include "clipper.h"
#include "clipper_core.hpp"
//#include <list>

namespace clipperlib {

enum class JoinType { Square, Round, Miter };

enum class EndType {Polygon, Joined, Butt, Square, Round};
//Butt   : offsets both sides of a path, with square blunt ends
//Square : offsets both sides of a path, with square extended ends
//Round  : offsets both sides of a path, with round extended ends
//Joined : offsets both sides of a path, with joined ends
//Polygon: offsets only one side of a closed path

class PathGroup {
public:
	PathsD paths;
	JoinType join_type;
	EndType end_type;
	PathGroup(const PathsD &paths, JoinType join_type, EndType end_type):
		paths(paths), join_type(join_type), end_type(end_type) {}
};

class ClipperOffset {
private:
	double delta = 0.0;
	double sin_val = 0.0;
	double cos_val = 0.0;
	double miter_lim = 0.0; 
	double miter_limit = 0.0;
	double steps_per_rad = 0.0;
	PathD norms;
	std::vector<PathGroup*> group_in;
	PathD path_in;
	PathD path_out;
	PathsD paths_out;
	PathsD solution;
	JoinType join_type = JoinType::Square;
	double arc_tolerance = 0.0;
	double min_edge_len_sqrd = 0.0;
	double min_edge_len = 0.25;
	double scale_ = 1.0;
	bool merge_groups = false;
	inline void AddPoint(double x, double y) { path_out.push_back(PointD(x, y)); };
	inline void AddPoint(PointD pt) { path_out.push_back(pt); };
	void DoSquare(size_t j, size_t k);
	void DoMiter(size_t j, size_t k, double cosAplus1);
	void DoRound(size_t j, size_t k, double angle);
	void BuildNormals();
	void OffsetPolygon();
	void OffsetOutline();
	void OffsetOpenLine(EndType et);
	void OffsetPoint(size_t j, size_t &k);
	void DoOffset(PathGroup &pathGroup, double delta_);
public:
	ClipperOffset(double miter_limit_ = 2.0, double arc_tolerance_ = 0.0) :
		miter_limit(miter_limit_), arc_tolerance(arc_tolerance_) {};
	~ClipperOffset() { Clear(); };

	void AddPath(const PathD &p, JoinType jt_, EndType et_);
	void AddPaths(const PathsD &p, JoinType jt_, EndType et_);
	void Clear();
	PathsD Execute(double delta_);

	//ArcTolerance: needed for rounded offsets (See offset_triginometry2.svg)
	void ArcTolerance(double arc_tolerance_) { arc_tolerance = arc_tolerance_; }
	//MergeGroups: A path group is one or more paths added via the AddPath or
	//AddPaths methods. By default these path groups will be offset
	//independently of other groups and this may cause overlaps (intersections).
	//However, when MergeGroups is enabled, any overlapping offsets will be
	//merged (via a clipping union operation) to remove overlaps.
	void MergeGroups(bool merge_groups_) { merge_groups = merge_groups_; }
	//MinEdgeLength: Very small edges are often at unexpected angles and these
	//can become visible artefacts with offsetting. 
	//Consequently it's best to remove these tiny edges before processing.
	void MinEdgeLength(double min_edge_len_) { min_edge_len = min_edge_len_; }
};

PathsI InflatePaths(const PathsI &paths, double delta, JoinType jt, EndType et);
PathsD InflatePaths(const PathsD &paths, double delta, JoinType jt, EndType et);

const double default_arc_frac = 0.025;
const double two_pi = 2.0 * PI;
const double quarter_pi = 0.25 * PI;

}
#endif /* CLIPPER_OFFSET_H_ */
