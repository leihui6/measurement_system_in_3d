#pragma once

#include <CGAL/Simple_cartesian.h>
#include <CGAL/linear_least_squares_fitting_3.h>

#include "cloud_geometry.h"

typedef CGAL::Simple_cartesian<float> K;

class cloud_fitting
{
public:
	cloud_fitting();

	~cloud_fitting();

	void fitting_line_3d_linear_least_squares(std::vector<point_3d>& points, line_func_3d & line_func);


};

