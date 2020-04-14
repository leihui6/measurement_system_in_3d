#ifndef CLOUD_GEOMETRY_H
#define CLOUD_GEOMETRY_H

#include <CGAL/Simple_cartesian.h>
#include <CGAL/linear_least_squares_fitting_3.h>

#include "cloud_point.h"

struct line_func_3d
{
	line_func_3d();

	float x, y, z;

	float n, m, l;

	void set_xyz(float x, float y, float z);

	void set_nml(float n, float m, float l);
};

struct plane_func
{
	plane_func();

	float a, b, c, d;
};


struct cylinder_func
{
	cylinder_func();

	line_func_3d m_line_func;

	float r;
};

typedef CGAL::Simple_cartesian<float> K;

void fitting_line_3d_linear_least_squares(std::vector<point_3d>& points, line_func_3d & line_func);

#endif // !GEOMETRY_H
