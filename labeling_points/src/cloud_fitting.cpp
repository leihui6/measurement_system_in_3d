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

	float fitting_quality =
		linear_least_squares_fitting_3(cgal_points.begin(), cgal_points.end(), cgal_line_func, CGAL::Dimension_tag<0>());

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

void cloud_fitting::fitting_plane_3d_linear_least_squares(std::vector<point_3d>& points, plane_func_3d & plane_func)
{
	std::vector<K::Point_3> cgal_points;

	convert_to_CGAL_points(points, cgal_points);

	K::Plane_3 cgal_plane_func;

	float fitting_quality = 
		linear_least_squares_fitting_3(cgal_points.begin(), cgal_points.end(), cgal_plane_func, CGAL::Dimension_tag<0>());

	plane_func.set_abcd(
		cgal_plane_func.a(),
		cgal_plane_func.b(),
		cgal_plane_func.c(),
		cgal_plane_func.d()
	);
}

void cloud_fitting::get_convex_hull_from_points(std::vector<point_3d>& points, std::vector<point_3d>& convex_hull_points)
{
	typedef CGAL::Exact_predicates_inexact_constructions_kernel  EK;

	std::vector<EK::Point_3> cgal_points;

	for (size_t i = 0; i < points.size(); ++i)
	{
		EK::Point_3 p(points[i].x, points[i].y, points[i].z);

		cgal_points.push_back(p);
	}

	CGAL::Polyhedron_3<EK> poly;

	CGAL::convex_hull_3(cgal_points.begin(), cgal_points.end(), poly);

	convex_hull_points.resize(poly.size_of_vertices());

	size_t i = 0;
	for (CGAL::Polyhedron_3<CGAL::Exact_predicates_inexact_constructions_kernel>::Vertex_iterator it = poly.vertices_begin();
		it != poly.vertices_end();
		++it)
	{
		convex_hull_points[i].set_xyz(
			it->point()[0],
			it->point()[1],
			it->point()[2]
		);
		++i;
	}
}
