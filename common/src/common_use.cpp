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

		std::this_thread::sleep_for(std::chrono::milliseconds(400));

		if (i + 1 == transformation_vec.size())
		{
			i = 0;

			continue;
		}

		i++;
	}
}

void read_points(std::map<std::string, std::vector<point_3d>> & points_map, const std::string & file_name)
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
			points_map[line.substr(1, line.size() - 1)] = points;

			points.clear();

			continue;
		}

		std::stringstream s(line);

		float value[3] = { 0,0,0 };

		for (size_t i = 0; i < 3; i++)
		{
			s >> value[i];
		}
		points.push_back(point_3d(value[0], value[1], value[2]));
	}

	ifile.close();
}

void export_marked_points(std::map<std::string, std::vector<point_3d>>& marked_points, const std::string & export_file_name)
{
	std::map <std::string, std::vector<point_3d>>::iterator it;

	std::ofstream point_file(export_file_name);

	if (point_file.is_open())
	{
		for (it = marked_points.begin(); it != marked_points.end(); it++)
		{
			std::vector<point_3d> &ps = it->second;

			for (size_t j = 0; j < ps.size(); j++)
			{
				point_file << ps[j].x << " " << ps[j].y << " " << ps[j].z << "\n";
			}
			point_file << "#" << it->first << "\n";
		}
		point_file.close();
	}
}

void export_measured_data(std::multimap<std::string, std::string>& measurement_pairs_map, std::vector<measurement_value>& mv_vec, const std::string & output_file_name)
{
	std::ofstream  ofile(output_file_name);

	if (!ofile.is_open()) return;

	std::multimap<std::string, std::string>::iterator it;

	size_t i = 0;
	for (it = measurement_pairs_map.begin(); it != measurement_pairs_map.end(); ++it)
	{
		ofile << it->first << " " << it->second << " ";

		ofile << ((mv_vec[i].is_valid[0] == true) ? mv_vec[i].distance_geometry : -1) << " ";
		ofile << ((mv_vec[i].is_valid[1] == true) ? (mv_vec[i].distance_scattered[0]) : (-1.0f)) << " ";
		ofile << ((mv_vec[i].is_valid[1] == true) ? (mv_vec[i].distance_scattered[1]) : (-1.0f)) << " ";
		ofile << ((mv_vec[i].is_valid[2] == true) ? mv_vec[i].angle : -1);

		ofile << "\n";

		++i;
	}
	ofile.close();
}

void transform_marked_points(std::map<std::string, std::vector<point_3d>>& marked_points, Eigen::Matrix4f & m)
{
	std::map<std::string, std::vector<point_3d>>::iterator it;

	for (it = marked_points.begin(); it != marked_points.end(); it++)
	{
		std::vector<point_3d> & ptsv = it->second;

		for (size_t i = 0; i < ptsv.size(); ++i)
		{
			ptsv[i].do_transform(m);
		}
	}
}

void read_file_as_map(const std::string & file_name, std::map<std::string, std::string> & str_flt_map)
{
	std::fstream ifile;

	if (!open_file(file_name, &ifile)) return;

	std::string line;

	while (std::getline(ifile, line))
	{
		if (line.empty()) continue;

		if (line[0] == '#') continue;

		size_t divided_flag = line.find(":");

		if (divided_flag == std::string::npos) continue;

		std::string
			key_ = line.substr(0, divided_flag++);

		std::string
			value_ = line.substr(divided_flag, line.size() - divided_flag);

		str_flt_map[key_] = value_;
	}
	std::cout << "read " << str_flt_map.size() << " parameters from local file.\n";
}

void read_file_as_map(const std::string & file_name, std::multimap<std::string, std::string> & str_flt_map)
{
	std::fstream ifile(file_name);

	if (!open_file(file_name, &ifile)) return;

	std::string line;

	while (std::getline(ifile, line))
	{
		if (line.empty()) break;

		if (line[0] == '#') continue;

		size_t divided_flag = line.find(":");

		if (divided_flag == std::string::npos) continue;

		std::string
			key_ = line.substr(0, divided_flag++);

		std::string
			value_ = line.substr(divided_flag, line.size() - divided_flag);

		str_flt_map.insert(std::pair<std::string, std::string>(key_, value_));
	}
}

bool open_file(const std::string & file_name, std::fstream * f, bool clear)
{
	try
	{
		if (clear)
			f->open(file_name, std::fstream::in | std::fstream::out);
		else
			f->open(file_name, std::fstream::in | std::fstream::out | std::fstream::app);
	}
	catch (const std::exception&)
	{
		std::cerr << "[error]" << "file path:\"" << file_name << "\" does not exist.\n";
		f = nullptr;
		return false;
	}
	return true;
}

osg::Vec4 str_to_vec4(const std::string & s)
{
	std::stringstream ss(s);
	osg::Vec4 vec4;
	float tmp; size_t i = 0;
	while (ss >> tmp) vec4[i++] = tmp;
	if (i != 4)	return osg::Vec4();
	else	return vec4;
}

void save_matrix(Eigen::Matrix4f & matrix, const std::string & file_name)
{
	std::ofstream file(file_name);

	if (file.is_open())
	{
		file << matrix << "\n#\n";
	}

	file.close();
}

void read_matrix(const std::string & file_name, std::vector<Eigen::Matrix4f> & m_v)
{
	std::ifstream ifile(file_name);

	if (!ifile.is_open())
	{
		std::cerr << "file (" << file_name << ") doesn't exist\n";
		return;
	}
	std::string line;

	std::vector<float> matrix_value;

	while (std::getline(ifile, line))
	{
		if (line.empty())
		{
			continue;
		}

		if (line[0] == '#')
		{
			continue;
		}

		std::stringstream s(line);

		float tmp;

		for (size_t i = 0; i < 4; ++i)
		{
			s >> tmp;
			matrix_value.push_back(tmp);
		}

		if (matrix_value.size() == 16)
		{
			Eigen::Matrix4f m_tmp(matrix_value.data());

			m_tmp.transposeInPlace();

			m_v.push_back(m_tmp);

			matrix_value.clear();
		}
	}

	ifile.close();
}