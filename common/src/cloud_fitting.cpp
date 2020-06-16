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

void cloud_fitting::fitting_cylinder_ransac(std::vector<point_3d>& points, cylinder_func & _cylinder_func, size_t iteration_count)
{
	//_cylinder_func.r = 0.0;

	//std::vector<size_t> random_vec(points.size());

	//for (size_t i = 0; i < random_vec.size(); ++i)
	//{
	//	random_vec[i] = i;
	//}

	//int i = 0;

	//float min_deviation = FLT_MAX, required_probability = 0.8;

	////float cylinder_r = 0;

	////point_3d cylinder_line_point, cylinder_line_direction;

	//while (true)
	//{
	//	if (i++ > iteration_count)
	//	{
	//		break;
	//	}

	//	std::random_shuffle(random_vec.begin(), random_vec.end());

	//	//std::cout << random_vec[0] << " " << random_vec[1] << " " << random_vec[2] << std::endl;

	//	plane_func_3d plane_func;

	//	plane_function_from_three_points(points[random_vec[0]], points[random_vec[1]], points[random_vec[2]], plane_func);

	//	std::vector<point_3d> use_for_cylinder;

	//	points_on_plane(points, use_for_cylinder, plane_func, 0.2);

	//	point_3d centriod_point;

	//	centroid_from_points(use_for_cylinder, centriod_point);

	//	float mean_distance = 0.0;

	//	mean_distance_from_point_to_points(use_for_cylinder, centriod_point, mean_distance);

	//	//std::cout << "mean_distance:" << mean_distance << std::endl;

	//	line_func_3d line_func;

	//	line_func.set_xyz(centriod_point.x, centriod_point.y, centriod_point.z);

	//	line_func.set_nml(plane_func.a, plane_func.b, plane_func.c);

	//	std::vector<float> distance_to_cylinder_line;

	//	distance_points_to_line(use_for_cylinder, line_func, distance_to_cylinder_line);

	//	float deviation = 0.0;// , probability = 0.0;

	//	//probability_close_to_value(distance_to_cylinder_line, mean_distance, 0.1, probability);

	//	standard_deviation(distance_to_cylinder_line, deviation);

	//	if (deviation < min_deviation)
	//	//if (probability > required_probability)
	//	{
	//		min_deviation = deviation;

	//		_cylinder_func.r = mean_distance;

	//		_cylinder_func.m_line_func.set_xyz(centriod_point.x, centriod_point.y, centriod_point.z);

	//		_cylinder_func.m_line_func.set_nml(plane_func.a, plane_func.b, plane_func.c);

	//		//required_probability = probability;
	//	}
	//	else
	//	{
	//		std::cout
	//			<< "deviation=" << deviation << "\n";
	//	}
	//}
	//std::cout
	//	<< "min_deviation=" << min_deviation << "\n"
	//	<< "cylinder_r=" << _cylinder_func.r << "\n";
}

/*
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
*/
