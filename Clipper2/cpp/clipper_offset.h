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

#ifndef CLIPPER_OFFSET_H_
#define CLIPPER_OFFSET_H_

#include "clipper_core.h"
#include <list>

namespace clipperlib {

enum class JoinType { Square, Round, Miter };
enum class EndType {Polygon, OpenJoined, OpenButt, OpenSquare, OpenRound};

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
	JoinType join_type;
	double arc_tolerance;

	double scale_ = 1.0;
	inline void addPoint(double x, double y) { path_out.push_back(PointD(x, y)); };
	inline void addPoint(PointD pt) { path_out.push_back(pt); };
	void doSquare(size_t j, size_t k);
	void doMiter(size_t j, size_t k, double cosAplus1);
	void doRound(size_t j, size_t k, double angle);
	void buildNormals();
	void offsetPolygon();
	void offsetOpenJoined();
	void offsetOpenPath(EndType et);
	void offsetPoint(size_t j, size_t &k);
	void doOffset(PathGroup &pathGroup, double delta_);
public:
	ClipperOffset(double miter_limit_ = 2.0, double arc_tolerance_ = 0.0);
	~ClipperOffset();

	void addPath(const PathD &p, JoinType jt_, EndType et_);
	void addPaths(const PathsD &p, JoinType jt_, EndType et_);
	void clear();
	PathsD execute(double delta_);
};

PathsI clipperOffsetPaths(const PathsI paths, double delta, JoinType jt, EndType et);
PathsD clipperOffsetPaths(const PathsD paths, double delta, JoinType jt, EndType et);

const double tolerance = 1E-15;
const double default_arc_frac = 0.05;
const double two_pi = 2.0 * 3.141592653589793238;
const double quarter_pi = 0.25 * 3.141592653589793238;
}
#endif /* CLIPPER_OFFSET_H_ */
