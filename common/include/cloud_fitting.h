#ifndef CLOUD_FITTING_H
#define CLOUD_FITTING_H

#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>

#include "cloud_geometry.h"

typedef CGAL::Simple_cartesian<float> K;

class cloud_fitting
{
public:
	cloud_fitting();

	~cloud_fitting();

	void fitting_line_3d_linear_least_squares(std::vector<point_3d>& points, line_func_3d & line_func);

	void fitting_plane_3d_linear_least_squares(std::vector<point_3d>& points, plane_func_3d & plane_func);

	void fitting_cylinder_ransac(std::vector<point_3d>& points, cylinder_func & _cylinder_func, size_t iteration_count = 20);

	//void get_convex_hull_from_points(std::vector<point_3d>& points, std::vector<point_3d> & convex_hull_points);
};

#endif //CLOUD_FITTING_H