#include "cloud_fitting.h"

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
