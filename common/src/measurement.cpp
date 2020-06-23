#include "measurement.h"

void measure(std::string & points_1_name, std::string & points_2_name, std::map<std::string, std::vector<point_3d>>& _m, measurement_value & mv)
{
	if (_m.find(points_1_name) == std::end(_m))
	{
		std::cerr << "No " << points_1_name << " name, please check it again.\n";
		return;
	}

	if (_m.find(points_2_name) == std::end(_m))
	{
		std::cerr << "No " << points_2_name << " name, please check it again.\n";
		return;
	}

	std::vector<size_t> upper(4, 0), lower(4, 0);

	if (points_1_name.find("point"))
	{
		upper[0] = 1;
	}
	else if (points_1_name.find("line"))
	{
		upper[1] = 1;
	}
	else if (points_1_name.find("plane"))
	{
		upper[2] = 1;
	}
	else if (points_1_name.find("cylinder"))
	{
		upper[3] = 1;
	}

	if (points_2_name.find("point"))
	{
		lower[0] = 1;
	}
	else if (points_2_name.find("line"))
	{
		lower[1] = 1;
	}
	else if (points_2_name.find("plane"))
	{
		lower[2] = 1;
	}
	else if (points_2_name.find("cylinder"))
	{
		lower[3] = 1;
	}

	analyze_points(upper, lower, _m[points_1_name], _m[points_2_name], mv);
}

void analyze_points(std::vector<size_t>& order_1, std::vector<size_t>& order_2, std::vector<point_3d> points_1, std::vector<point_3d> points_2, measurement_value & mv)
{
	size_t first_type, second_type;

	for (size_t i = 0; i < order_1.size(); ++i)
	{
		if (order_1[i] != 0)
		{
			first_type = i;
		}
		if (order_2[i] != 0)
		{
			second_type = i;
		}
	}

	if (first_type == 0 && second_type == 0)
	{
	}
	else if (first_type == 0 && second_type == 1)
	{
	}
	else if (first_type == 0 && second_type == 2)
	{
	}
	else if (first_type == 0 && second_type == 3)
	{
	}
	else if (first_type == 0 && second_type == 3)
	{
	}
	else if (first_type == 1 && second_type == 0)
	{
	}
	else if (first_type == 1 && second_type == 1)
	{
	}
	else if (first_type == 1 && second_type == 2)
	{
	}
	else if (first_type == 1 && second_type == 3)
	{
	}
	else if (first_type == 2 && second_type == 0)
	{
	}
	else if (first_type == 2 && second_type == 1)
	{
	}
	else if (first_type == 2 && second_type == 2)
	{
	}
	else if (first_type == 2 && second_type == 3)
	{
	}
	else if (first_type == 3 && second_type == 0)
	{
	}
	else if (first_type == 3 && second_type == 1)
	{
	}
	else if (first_type == 3 && second_type == 2)
	{
	}
	else if (first_type == 3 && second_type == 3)
	{
	}
}
