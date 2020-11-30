#include "cloud_io.h"

//! read point data from files
/*
	//...
*/
bool load_point_cloud_txt(const std::string & filename, std::vector<point_3d>& points, bool fill_normals)
{
	points.clear();

	std::ifstream ifile(filename, std::ios::in);

	if (!ifile.is_open())
	{
		return false;
	}
	std::string line;

	point_3d point;

	while (std::getline(ifile, line))
	{
		if (line.size() < 1)
		{
			continue;
		}

		if (line[0] == '#')
		{
			continue;
		}

		std::stringstream s(line);

		float tmp = 0.0, value[9] = { 0 };

		int i = 0;

		while (s >> tmp)
		{
			value[i] = tmp;

			i++;
		}

		point.set_xyz(value[0], value[1], value[2]);
		point.set_nxyz(value[3], value[4], value[5]);
		point.set_rgb(value[6], value[7], value[8]);

		points.push_back(point);
	}

	//std::cout << "load_point_cloud_txt() " << filename << " size=" << points.size() << std::endl;

	if (fill_normals)
	{
		cloud_processing cp;

		cp.estimate_normals_with_k(points, 7);

		std::cout << "estimating normals with points as required" << std::endl;
	}

	return true;
}

bool load_point_cloud_vtk(const std::string & filename, std::vector<point_3d>& points)
{
	points.clear();

	std::ifstream ifile(filename, std::ios::in);

	if (!ifile.is_open())
	{
		return false;
	}
	std::string line;

	point_3d point;

	bool readable = false;

	size_t data_size = 0;

	while (std::getline(ifile, line))
	{
		if (line.size() < 1)
		{
			continue;
		}

		if (line[0] == '#')
		{
			continue;
		}

		std::stringstream s(line);

		if (readable)
		{
			float tmp = 0.0, value[9] = { 0,0,0,0,0,0,0,0,0 };

			int i = 0;

			while (s)
			{
				s >> tmp;

				value[i] = tmp;

				i++;
			}

			point.set_xyz(value[0], value[1], value[2]);
			point.set_nxyz(value[3], value[4], value[5]);
			point.set_rgb(value[6], value[7], value[8]);

			points.push_back(point);

			if (points.size() == data_size)
			{
				readable = false;
			}

			continue;
		}

		std::string header;

		s >> header;

		if (header == "POINTS")
		{
			s >> data_size;

			readable = true;

			continue;
		}
	}
}
