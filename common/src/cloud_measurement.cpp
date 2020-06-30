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

	mv.is_valid[0] = true;
	mv.is_valid[1] = true;
	mv.is_valid[2] = false;

	point_3d centroid_point_1, centroid_point_2;
	centroid_from_points(points_1, centroid_point_1);
	centroid_from_points(points_2, centroid_point_2);

	distance_point_to_point(centroid_point_1, centroid_point_2, mv.distance_geometry);

	distance_scattered_points(points_1, points_2, mv.distance_scattered[0], mv.distance_scattered[1]);
}	

void cloud_measurement::calculate_point_to_line(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, measurement_value & mv)
{
	std::cout << "calculate_point_to_line\n";

	mv.is_valid[0] = true;
	mv.is_valid[1] = false;
	mv.is_valid[2] = false;

	// calculate the geometrical distance.
	point_3d centroid_point_1;
	centroid_from_points(points_1, centroid_point_1);

	line_func_3d line_func;
	cf.fitting_line_3d_linear_least_squares(points_2, line_func);

	distance_point_to_line(centroid_point_1, line_func, mv.distance_geometry);
	/*
	// calculate the scattered distance.
	point_3d centroid_point_2;
	float loggest_distance = 0.0;
	centroid_from_points(points_2, centroid_point_2);
	longgest_distance_from_point_to_points(points_2, centroid_point_2, loggest_distance);

	point_3d intersection_p_1, intersection_p_2;
	intersection_line_to_sphere(line_func, centroid_point_2, loggest_distance, intersection_p_1, intersection_p_2);

	point_3d pedal_p;
	pedalpoint_point_to_line(centroid_point_1, line_func, pedal_p);

	float seg_dis_1 = 0.0, seg_dis_2 = 0.0;
	distance_point_to_point(centroid_point_1, intersection_p_1, seg_dis_1);
	distance_point_to_point(centroid_point_1, intersection_p_2, seg_dis_2);

	// pedel point is in segment points
	if (is_in_range_of_two_points(pedal_p, intersection_p_1, intersection_p_2))
	{
		distance_point_to_point(centroid_point_1, pedal_p, mv.distance_scattered[0]);

		if (seg_dis_1 > seg_dis_2)
			mv.distance_scattered[1] = seg_dis_1;
		else
			mv.distance_scattered[1] = seg_dis_2;
	}
	else
	{
		if (seg_dis_1 > seg_dis_2)
		{
			mv.distance_scattered[0] = seg_dis_2;
			mv.distance_scattered[1] = seg_dis_1;
		}
		else
		{
			mv.distance_scattered[0] = seg_dis_1;
			mv.distance_scattered[1] = seg_dis_2;
		}
	}
	*/
}

void cloud_measurement::calculate_point_to_plane(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, measurement_value & mv)
{
	std::cout << "calculate_point_to_plane\n";

	mv.is_valid[0] = true;
	mv.is_valid[1] = false;
	mv.is_valid[2] = false;

	point_3d centroid_point;
	centroid_from_points(points_1, centroid_point);

	plane_func_3d plane_func;
	cf.fitting_plane_3d_linear_least_squares(points_2, plane_func);

	distance_point_to_plane(centroid_point, plane_func, mv.distance_geometry);
}

void cloud_measurement::calculate_point_to_cylinder(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, measurement_value & mv)
{
	std::cout << "calculate_point_to_cylinder\n";

	mv.is_valid[0] = true;
	mv.is_valid[1] = false;
	mv.is_valid[2] = false;

	point_3d centroid_point;
	centroid_from_points(points_1, centroid_point);

	cylinder_func _cylinder_func;
	cf.fitting_cylinder_linear_least_squares(points_2, _cylinder_func);

	distance_point_to_line(centroid_point, _cylinder_func.axis, mv.distance_geometry);
}

void cloud_measurement::calculate_line_to_line(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, measurement_value & mv)
{
	std::cout << "calculate_line_to_line\n";

	mv.is_valid[0] = false;
	mv.is_valid[1] = false;
	mv.is_valid[2] = true;

	line_func_3d line_func_1, line_func_2;
	cf.fitting_line_3d_linear_least_squares(points_1, line_func_1);
	cf.fitting_line_3d_linear_least_squares(points_2, line_func_2);

	angle_between_two_vector_3d(line_func_1.direction, line_func_2.direction, mv.angle);

	if (mv.angle > 90)
		mv.angle = 180 - mv.angle;
}

void cloud_measurement::calculate_line_to_plane(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, measurement_value & mv)
{
	std::cout << "calculate_line_to_plane\n";

	mv.is_valid[0] = false;
	mv.is_valid[1] = false;
	mv.is_valid[2] = true;

	line_func_3d line_func;
	cf.fitting_line_3d_linear_least_squares(points_1, line_func);

	plane_func_3d plane_func;
	cf.fitting_plane_3d_linear_least_squares(points_2, plane_func);

	angle_between_two_vector_3d(line_func.direction, plane_func.direction<Eigen::Vector3f>(), mv.angle);

	if (mv.angle > 90)
		mv.angle = 180 - mv.angle;

	mv.angle = 90 - mv.angle;
}

void cloud_measurement::calculate_line_to_cylinder(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, measurement_value & mv)
{
	std::cout << "calculate_line_to_cylinder\n";

	mv.is_valid[0] = false;
	mv.is_valid[1] = false;
	mv.is_valid[2] = true;

	line_func_3d line_func;
	cf.fitting_line_3d_linear_least_squares(points_1, line_func);

	cylinder_func _cylinder_func;
	cf.fitting_cylinder_linear_least_squares(points_2, _cylinder_func);

	angle_between_two_vector_3d(line_func.get_direction_point_3d(), _cylinder_func.axis.get_direction_point_3d(), mv.angle);

	if (mv.angle > 90)
		mv.angle = 180 - mv.angle;
}

void cloud_measurement::calculate_plane_to_plane(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, measurement_value & mv)
{
	std::cout << "calculate_plane_to_plane\n";

	std::cout << "calculate_line_to_plane\n";

	mv.is_valid[0] = false;
	mv.is_valid[1] = false;
	mv.is_valid[2] = true;

	plane_func_3d plane_func_1, plane_func_2;
	cf.fitting_plane_3d_linear_least_squares(points_1, plane_func_1);
	cf.fitting_plane_3d_linear_least_squares(points_2, plane_func_2);

	angle_between_two_vector_3d(plane_func_1.direction<Eigen::Vector3f>(), plane_func_2.direction<Eigen::Vector3f>(), mv.angle);
}

void cloud_measurement::calculate_plane_to_cylinder(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, measurement_value & mv)
{
	std::cout << "calculate_plane_to_cylinder\n";

	mv.is_valid[0] = false;
	mv.is_valid[1] = false;
	mv.is_valid[2] = true;

	plane_func_3d plane_func;
	cf.fitting_plane_3d_linear_least_squares(points_1, plane_func);

	cylinder_func _cylinder_func;
	cf.fitting_cylinder_linear_least_squares(points_2, _cylinder_func);

	angle_between_two_vector_3d(plane_func.direction<point_3d>(), _cylinder_func.axis.get_direction_point_3d(), mv.angle);

	if (mv.angle > 90)
		mv.angle = 180 - mv.angle;

	mv.angle = 90 - mv.angle;
}

void cloud_measurement::calculate_cylinder_to_cylinder(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, measurement_value & mv)
{
	std::cout << "calculate_cylinder_to_cylinder\n";

	mv.is_valid[0] = false;
	mv.is_valid[1] = false;
	mv.is_valid[2] = true;

	cylinder_func _cylinder_func_1, _cylinder_func_2;
	cf.fitting_cylinder_linear_least_squares(points_1, _cylinder_func_1);
	cf.fitting_cylinder_linear_least_squares(points_2, _cylinder_func_2);

	angle_between_two_vector_3d(_cylinder_func_1.axis.get_direction_point_3d(), _cylinder_func_2.axis.get_direction_point_3d(), mv.angle);

	if (mv.angle > 90)
		mv.angle = 180 - mv.angle;
}

void cloud_measurement::distance_scattered_points(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, float & min_distance, float & max_distance)
{
	float min_dis = FLT_MAX, max_dis = FLT_MIN;

	for (size_t i = 0; i < points_1.size(); ++i)
	{
		for (size_t j = 0; j < points_2.size(); ++j)
		{
			float dis = 0.0;

			distance_point_to_point(points_1[i], points_2[j], dis);

			if (dis < min_dis)
			{
				min_dis = dis;
			}

			if (dis > max_dis)
			{
				max_dis = dis;
			}
		}
	}

	//std::cout << "max=" << max_distance << " min=" << min_distance << "\n";

	min_distance = min_dis;

	max_distance = max_dis;
}
