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

void load_file_to_display(std::string & folder, cloud_viewer * cv, float r, float g, float b, float point_size, size_t interval_time)
{
	size_t i = 0;

	std::vector<std::vector<point_3d>> points_vec;

	bool use_loaded_data = false;

	size_t max_file_number = UINT_MAX;

	while (true)
	{
		if (use_loaded_data == false)
		{
			std::string file_name = folder + "/" + "-reading-" + std::to_string(i) + ".vtk";

			bool is_existed = std::filesystem::exists(file_name);

			if (is_existed)
			{
				std::cout << "loaded " << file_name << "\n";

				std::vector<point_3d> points;

				load_point_cloud_vtk(file_name, points);

				cv->update_reading_point_cloud(points, r, g, b, point_size);

				points_vec.push_back(points);

				std::this_thread::sleep_for(std::chrono::milliseconds(interval_time));

				++i;
			}
			else
			{
				std::cout << "loaded all files, here are animation of the icp registration as below.\n";

				use_loaded_data = true;

				max_file_number = i;

				i = 0;
			}
		}
		else
		{
			cv->update_reading_point_cloud(points_vec[i], 0, 255, 0, 4);

			std::this_thread::sleep_for(std::chrono::milliseconds(interval_time));

			++i;

			if (i == max_file_number)
			{
				i = 0;
			}
		}
	}
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
	return m;
}