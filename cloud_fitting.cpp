#include "cloud_fitting.h"

line_func_3d::line_func_3d()
	:x(0), y(0), z(0),
	n(0), m(0), l(0)
{

}

void line_func_3d::set_xyz(float x, float y, float z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

void line_func_3d::set_nml(float n, float m, float l)
{
	this->n = n;
	this->m = m;
	this->l = l;
}

plane_func::plane_func()
	: a(0), b(0), c(0), d(0)
{

}

cylinder_func::cylinder_func()
	: r(0)
{

}

cloud_fitting::cloud_fitting()
{
}


cloud_fitting::~cloud_fitting()
{

}

void cloud_fitting::fitting_line_3d_linear_least_squares(std::vector<point_3d>& points, line_func_3d & line_func)
{
	std::vector<K::Point_3> cgal_points;

	convert_to_CGAL_points(points, cgal_points);

	K::Line_3 cgal_line_func;

	float fitting_quality = linear_least_squares_fitting_3(cgal_points.begin(), cgal_points.end(), cgal_line_func, CGAL::Dimension_tag<0>());

	//std::cout << fitting_quality << std::endl;

	//std::cout << cgal_line_func << std::endl;
	//std::cout << cgal_line_func.point() << std::endl;
	//std::cout << cgal_line_func.to_vector() << std::endl;

	//std::cout << cgal_line_func.point()[0] << std::endl;
	//std::cout << cgal_line_func.point()[1] << std::endl;
	//std::cout << cgal_line_func.point()[2] << std::endl;
	//std::cout << cgal_line_func.to_vector()[0] << std::endl;
	//std::cout << cgal_line_func.to_vector()[1] << std::endl;
	//std::cout << cgal_line_func.to_vector()[2]<< std::endl;
	//std::cout << cgal_line_func.point(2) << std::endl;

	line_func.set_xyz(
		cgal_line_func.point()[0],
		cgal_line_func.point()[1],
		cgal_line_func.point()[2]
	);

	line_func.set_nml(
		cgal_line_func.to_vector()[0],
		cgal_line_func.to_vector()[1],
		cgal_line_func.to_vector()[2]);
}
