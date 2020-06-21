#include "common_use.h"

std::string file_name_without_postfix(std::string & file_name)
{
	size_t first_pos = file_name.find('/', 0), second_pos = file_name.find('.', 0);

	if (second_pos > first_pos)
	{
		return file_name.substr(first_pos + 1, second_pos - first_pos - 1);
	}
	return file_name;
}

bool is_exist(const std::string & file_name)
{
	return std::filesystem::exists(file_name);;
}

void load_file_to_vec(std::string & folder, std::vector<Eigen::Matrix4f> & m_vec)
{
	size_t i = 0;

	while (true)
	{
		std::string file_name = folder + "/" + "-matrix-" + std::to_string(i) + ".txt";

		if (is_exist(file_name))
		{
			std::cout << "loaded " << file_name << "\n";

			m_vec.push_back(read_matrix(file_name));
		}
		else
		{
			break;
		}
		++i;
	}
}

void display_point_cloud_from_transformation_vec(cloud_viewer & cv, std::vector<point_3d>& reading_point_cloud, std::vector<Eigen::Matrix4f>& transformation_vec)
{
	// transformation_vec[1] is a identify matrix
	std::swap(transformation_vec[0], transformation_vec[1]);

	std::vector<std::vector<point_3d>> transformed_points;

	for (size_t i = 2; i < transformation_vec.size(); ++i)
	{
		transformation_vec[i] = transformation_vec[i] * transformation_vec[1];
	}

	for (size_t i = 0; i < transformation_vec.size(); ++i)
	{
		std::vector<point_3d> t_points;

		transform_points(reading_point_cloud, transformation_vec[i], t_points);

		transformed_points.push_back(t_points);
	}

	size_t i = 0;

	while (true)
	{
		cv.update_reading_point_cloud(transformed_points[i], 0, 255, 0, 4.0);

		std::this_thread::sleep_for(std::chrono::milliseconds(500));

		if (i + 1 == transformation_vec.size())
		{
			i = 0;

			continue;
		}

		i++;
	}
}

void read_points(std::vector<std::vector<point_3d>>& points_vec, const std::string & file_name)
{
	if (!is_exist(file_name))
	{
		std::cerr << "file (" << file_name << ") doesn't exist.\n";
		return;
	}

	std::ifstream ifile(file_name);
	std::string line;

	std::vector<point_3d> points;

	while (std::getline(ifile, line))
	{
		if (line.size() < 1)
		{
			continue;
		}

		if (line[0] == '#')
		{
			points_vec.push_back(points);

			points.clear();

			continue;
		}

		std::stringstream s(line);

		float value[3] = { 0,0,0 };

		for (size_t i = 0; i < 3; i++)
		{
			s >> value[i];
		}
		points.push_back(point_3d(value[0], value[2], value[2]));
	}

	ifile.close();
}

void save_matrix(Eigen::Matrix4f & matrix, const std::string & file_name)
{
	std::ofstream file(file_name);

	if (file.is_open())
	{
		file << matrix << '\n';
	}

	file.close();
}

Eigen::Matrix4f read_matrix(const std::string & file_name)
{
	Eigen::Matrix4f m;
	std::ifstream file(file_name);
	if (file.is_open())
	{
		std::vector<float> matrix_value;
		float tmp;

		while (file >> tmp)
		{
			matrix_value.push_back(tmp);
		}
		if (matrix_value.size() == 16)
		{
			Eigen::Matrix4f m_tmp(matrix_value.data());
			m = m_tmp;
		}
		else
		{
			std::cerr << "cannot read matrix from file:file error\n";
		}
	}
	else
	{
		std::cerr << "file (" << file_name << ") doesn't exist\n";
	}
	file.close();
	m.transposeInPlace();
	return m;
}