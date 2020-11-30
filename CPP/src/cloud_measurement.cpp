#include "cloud_measurement.h"

cloud_measurement::cloud_measurement()
{
}

cloud_measurement::~cloud_measurement()
{
}

void cloud_measurement::measure(std::multimap<std::string, std::string> measurement_pairs_map, std::map<std::string, std::vector<point_3d>>& _m, std::vector<measurement_value> & mv_vec)
{
	std::map<std::string, std::string>::iterator it;

	for (it = measurement_pairs_map.begin(); it != measurement_pairs_map.end(); ++it)
	{
		std::string
			points_1 = it->first,
			points_2 = it->second;

		if (_m.find(points_1) == std::end(_m)) continue;

		if (_m.find(points_2) == std::end(_m)) continue;

		measurement_value mv;

		measure(points_1, points_2, _m, mv);

		mv_vec.push_back(mv);
	}
}

void cloud_measurement::analyse_defect(std::vector<point_3d>& scanned_points, std::vector<point_3d>& standard_model, std::vector<point_3d> & defect_points)
{
	kd_tree kt(scanned_points);

	std::set<size_t> correspondences_index;
	kt.search_points_correspondence(standard_model, correspondences_index, 0.2);

	for (size_t i = 0; i < scanned_points.size(); ++i)
	{
		if (correspondences_index.find(i) == correspondences_index.end())
			defect_points.push_back(scanned_points[i]);
	}
}

void cloud_measurement::measure(std::string & points_1_name, std::string & points_2_name, std::map<std::string, std::vector<point_3d>>& _m, measurement_value & mv)
{
	if (_m.find(points_1_name) == std::end(_m)) return;

	if (_m.find(points_2_name) == std::end(_m)) return;

	std::vector<size_t> upper(4, 0), lower(4, 0);

	if (points_1_name.find("point") != std::string::npos)
		upper[0] = 1;
	else if (points_1_name.find("line") != std::string::npos)
		upper[1] = 1;
	else if (points_1_name.find("plane") != std::string::npos)
		upper[2] = 1;
	else if (points_1_name.find("cylinder") != std::string::npos)
		upper[3] = 1;

	if (points_2_name.find("point") != std::string::npos)
		lower[0] = 1;
	else if (points_2_name.find("line") != std::string::npos)
		lower[1] = 1;
	else if (points_2_name.find("plane") != std::string::npos)
		lower[2] = 1;
	else if (points_2_name.find("cylinder") != std::string::npos)
		lower[3] = 1;

	analyze_points(upper, lower, _m[points_1_name], _m[points_2_name], mv);
}

void cloud_measurement::analyze_points(std::vector<size_t>& order_1, std::vector<size_t>& order_2, std::vector<point_3d> points_1, std::vector<point_3d> points_2, measurement_value & mv)
{
	size_t first_type, second_type;

	for (size_t i = 0; i < order_1.size(); ++i)
	{
		if (order_1[i] != 0)
			first_type = i;

		if (order_2[i] != 0)
			second_type = i;
	}
	// point
	if (first_type == 0 && second_type == 0)
	{
		calculate_point_to_point(points_1, points_2, mv);
	}
	else if (first_type == 0 && second_type == 1)
	{
		calculate_point_to_line(points_1, points_2, mv);
	}
	else if (first_type == 0 && second_type == 2)
	{
		calculate_point_to_plane(points_1, points_2, mv);
	}
	else if (first_type == 0 && second_type == 3)
	{
		calculate_point_to_cylinder(points_1, points_2, mv);
	}
	// line
	// same as the 0->1, but exchange the parameters
	else if (first_type == 1 && second_type == 0)
	{
		calculate_point_to_line(points_2, points_1, mv);
	}
	else if (first_type == 1 && second_type == 1)
	{
		calculate_line_to_line(points_1, points_2, mv);
	}
	else if (first_type == 1 && second_type == 2)
	{
		calculate_line_to_plane(points_1, points_2, mv);
	}
	else if (first_type == 1 && second_type == 3)
	{
		calculate_line_to_cylinder(points_1, points_2, mv);
	}
	// plane
	// same as the 0->2, but exchange the parameters
	else if (first_type == 2 && second_type == 0)
	{
		calculate_point_to_plane(points_2, points_1, mv);
	}
	// same as the 1->2, but exchange the parameters
	else if (first_type == 2 && second_type == 1)
	{
		calculate_line_to_plane(points_2, points_1, mv);
	}
	else if (first_type == 2 && second_type == 2)
	{
		calculate_plane_to_plane(points_1, points_2, mv);
	}
	else if (first_type == 2 && second_type == 3)
	{
		calculate_plane_to_cylinder(points_1, points_2, mv);
	}
	// cylinder
	// same as the 0->3, but exchange the parameters
	else if (first_type == 3 && second_type == 0)
	{
		calculate_point_to_cylinder(points_2, points_1, mv);
	}
	// same as the 1->3, but exchange the parameters
	else if (first_type == 3 && second_type == 1)
	{
		calculate_line_to_cylinder(points_2, points_1, mv);
	}
	// same as the 2->3, but exchange the parameters
	else if (first_type == 3 && second_type == 2)
	{
		calculate_plane_to_cylinder(points_2, points_1, mv);
	}
	else if (first_type == 3 && second_type == 3)
	{
		calculate_cylinder_to_cylinder(points_1, points_2, mv);
	}
}

void cloud_measurement::calculate_point_to_point(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, measurement_value & mv)
{
	std::cout << "calculate_point_to_point\n";

	point_3d centroid_point_1, centroid_point_2;
	centroid_from_points(points_1, centroid_point_1);
	centroid_from_points(points_2, centroid_point_2);

	// geometrical distance
	distance_point_to_point(centroid_point_1, centroid_point_2, mv.distance_geometrical);
	
	// scattered distance
	distance_scattered_points(points_1, points_2, mv.distance_scattered);
}

void cloud_measurement::calculate_point_to_line(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, measurement_value & mv)
{
	std::cout << "calculate_point_to_line\n";

	// calculate the geometrical distance.
	point_3d centroid_point_1;
	centroid_from_points(points_1, centroid_point_1);

	line_func_3d line_func;
	cf.fitting_line_3d_linear_least_squares(points_2, line_func);

	// geometrical distance
	distance_point_to_line(centroid_point_1, line_func, mv.distance_geometrical);

	// scattered distance
	distance_scattered_points(points_1, points_2, line_func, mv.distance_scattered);
}

void cloud_measurement::calculate_point_to_plane(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, measurement_value & mv)
{
	std::cout << "calculate_point_to_plane\n";

	point_3d centroid_point;
	centroid_from_points(points_1, centroid_point);

	plane_func_3d plane_func;
	cf.fitting_plane_3d_linear_least_squares(points_2, plane_func);

	// geometrical distance
	distance_point_to_plane(centroid_point, plane_func, mv.distance_geometrical);

	// scattered distance
	distance_scattered_points(points_1, points_2, plane_func, mv.distance_scattered);
}

void cloud_measurement::calculate_point_to_cylinder(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, measurement_value & mv)
{
	std::cout << "calculate_point_to_cylinder\n";

	point_3d centroid_point;
	centroid_from_points(points_1, centroid_point);

	cylinder_func _cylinder_func;
	cf.fitting_cylinder_linear_least_squares(points_2, _cylinder_func);

	// geometrical distance
	distance_point_to_line(centroid_point, _cylinder_func.axis, mv.distance_geometrical);
}

void cloud_measurement::calculate_line_to_line(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, measurement_value & mv)
{
	std::cout << "calculate_line_to_line\n";

	line_func_3d line_func_1, line_func_2;
	cf.fitting_line_3d_linear_least_squares(points_1, line_func_1);
	cf.fitting_line_3d_linear_least_squares(points_2, line_func_2);

	// angle
	angle_between_two_vector_3d(line_func_1.direction, line_func_2.direction, mv.angle);
	if (mv.angle > 90)
		mv.angle = 180 - mv.angle;

	// scattered distance
	distance_scattered_points(points_1, line_func_1, points_2, line_func_2, mv.distance_scattered);
}

void cloud_measurement::calculate_line_to_plane(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, measurement_value & mv)
{
	std::cout << "calculate_line_to_plane\n";

	line_func_3d line_func;
	cf.fitting_line_3d_linear_least_squares(points_1, line_func);

	plane_func_3d plane_func;
	cf.fitting_plane_3d_linear_least_squares(points_2, plane_func);

	// angle
	angle_between_two_vector_3d(line_func.direction, plane_func.direction<Eigen::Vector3f>(), mv.angle);
	if (mv.angle > 90)
		mv.angle = 180 - mv.angle;

	mv.angle = 90 - mv.angle;

	// scattered distance
	distance_scattered_points(points_1, line_func, points_2, plane_func, mv.distance_scattered);
}

void cloud_measurement::calculate_line_to_cylinder(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, measurement_value & mv)
{
	std::cout << "calculate_line_to_cylinder\n";

	line_func_3d line_func;
	cf.fitting_line_3d_linear_least_squares(points_1, line_func);

	cylinder_func _cylinder_func;
	cf.fitting_cylinder_linear_least_squares(points_2, _cylinder_func);

	// angle
	angle_between_two_vector_3d(line_func.get_direction_point_3d(), _cylinder_func.axis.get_direction_point_3d(), mv.angle);
	if (mv.angle > 90)
		mv.angle = 180 - mv.angle;

	// geometrical distance(point(cylinder) to line)
	distance_point_to_line(_cylinder_func.axis.get_origin_point_3d(), line_func, mv.distance_geometrical);
}

void cloud_measurement::calculate_plane_to_plane(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, measurement_value & mv)
{
	std::cout << "calculate_plane_to_plane\n";

	plane_func_3d plane_func_1, plane_func_2;
	cf.fitting_plane_3d_linear_least_squares(points_1, plane_func_1);
	cf.fitting_plane_3d_linear_least_squares(points_2, plane_func_2);

	// angle
	angle_between_two_vector_3d(plane_func_1.direction<Eigen::Vector3f>(), plane_func_2.direction<Eigen::Vector3f>(), mv.angle);

	// scattered distance
	distance_scattered_points(points_1, plane_func_1, points_2, plane_func_2, mv.distance_scattered);
	//std::cout << mv.distance_scattered << std::endl;
}

void cloud_measurement::calculate_plane_to_cylinder(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, measurement_value & mv)
{
	std::cout << "calculate_plane_to_cylinder\n";

	plane_func_3d plane_func;
	cf.fitting_plane_3d_linear_least_squares(points_1, plane_func);

	cylinder_func _cylinder_func;
	cf.fitting_cylinder_linear_least_squares(points_2, _cylinder_func);

	// angle
	angle_between_two_vector_3d(plane_func.direction<point_3d>(), _cylinder_func.axis.get_direction_point_3d(), mv.angle);
	if (mv.angle > 90)
		mv.angle = 180 - mv.angle;

	mv.angle = 90 - mv.angle;

	// geometrical distance(point(cylinder) to plane)
	distance_point_to_plane(_cylinder_func.axis.get_origin_point_3d(), plane_func, mv.distance_geometrical);
}

void cloud_measurement::calculate_cylinder_to_cylinder(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, measurement_value & mv)
{
	std::cout << "calculate_cylinder_to_cylinder\n";

	cylinder_func _cylinder_func_1, _cylinder_func_2;
	cf.fitting_cylinder_linear_least_squares(points_1, _cylinder_func_1);
	cf.fitting_cylinder_linear_least_squares(points_2, _cylinder_func_2);

	// angle
	angle_between_two_vector_3d(_cylinder_func_1.axis.get_direction_point_3d(), _cylinder_func_2.axis.get_direction_point_3d(), mv.angle);
	if (mv.angle > 90)
		mv.angle = 180 - mv.angle;
	
	// geometrical distance(point(cylinder) to line(cylinder))
	distance_point_to_line(_cylinder_func_1.axis.get_origin_point_3d(), _cylinder_func_2.axis, mv.distance_geometrical);
}

void cloud_measurement::distance_scattered_points(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, float & distance)
{
	float min_dis = FLT_MAX;

	for (size_t i = 0; i < points_1.size(); ++i)
	{
		for (size_t j = 0; j < points_2.size(); ++j)
		{
			float dis = 0.0;
			distance_point_to_point(points_1[i], points_2[j], dis);

			if (dis < min_dis)
				min_dis = dis;
		}
	}
	distance = min_dis;
}

void cloud_measurement::distance_scattered_points(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, line_func_3d lf_2, float & distance)
{
	std::vector<point_3d> points_line_2;
	points_line(lf_2, points_2, points_line_2);

	distance_scattered_points(points_1, points_line_2, distance);
}

void cloud_measurement::distance_scattered_points(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, plane_func_3d & plane_func_2, float & distance)
{
	std::vector<point_3d> points_plane_2;
	points_plane(plane_func_2, points_2, points_plane_2);

	save_points(points_plane_2, "output/plane.txt");

	distance_scattered_points(points_1, points_plane_2, distance);
}

void cloud_measurement::distance_scattered_points(
	std::vector<point_3d>& points_1, line_func_3d &lf_1,
	std::vector<point_3d>& points_2, line_func_3d &lf_2,
	float & distance)
{
	std::vector<point_3d> points_line_1;
	points_line(lf_1, points_1, points_line_1);

	std::vector<point_3d> points_line_2;
	points_line(lf_2, points_2, points_line_2);

	distance_scattered_points(points_line_1, points_line_2, distance);
}

void cloud_measurement::distance_scattered_points(std::vector<point_3d>& points_1, line_func_3d & lf_1, std::vector<point_3d>& points_2, plane_func_3d & plane_func_2, float & distance)
{
	std::vector<point_3d> points_line_1;
	points_line(lf_1, points_1, points_line_1);

	std::vector<point_3d> points_plane_2;
	points_plane(plane_func_2, points_2, points_plane_2);

	distance_scattered_points(points_line_1, points_plane_2, distance);
}

static int t_c = 0;

void cloud_measurement::distance_scattered_points(
	std::vector<point_3d>& points_1, plane_func_3d & plane_func_1, 
	std::vector<point_3d>& points_2, plane_func_3d & plane_func_2,
	float & distance)
{
	std::vector<point_3d> points_plane_1;
	points_plane(plane_func_1, points_1, points_plane_1);
	//save_points(points_plane_1, "output/plane_" + std::to_string(++t_c) + ".txt");

	std::vector<point_3d> points_plane_2;
	points_plane(plane_func_2, points_2, points_plane_2);
	//save_points(points_plane_1, "output/plane_" + std::to_string(++t_c) + ".txt");

	distance_scattered_points(points_plane_1, points_plane_2, distance);
}
