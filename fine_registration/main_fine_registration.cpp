/*
*	The purpose of this program is to implement the fine registration
*
*	Author: Leihui Li
*	Date: 06/06/2020
*/

#define FINE_REGISTRATION

#ifdef FINE_REGISTRATION

#include <filesystem>

#include "cloud_io.h"
#include "cloud_viewer.h"
#include "cloud_fitting.h"
#include "cloud_registration.h"
#include "cloud_processing.h"

std::string file_name_without_postfix(std::string & file_name);

void load_file_to_display(std::string & folder, cloud_viewer * cv, float r = 0, float g = 255, float b = 0, float point_size = 4.0, size_t interval_time = 1000);

int main(int argc, char *argv[])
{
	if (argc < 5)
	{
		std::cerr
			//<< "please see help with \"--help\" \n "
			<< "This program needs two point cloud as input, a configruation file and a output folder\n"
			<< "you can enter more parameters like\n"
			<< "xxx.exe <reading_point_cloud.txt> <reference_point_cloud.txt> <configruation_file.yaml> <icp_output> \n";

		return -1;
	}

	std::string
		reading_point_cloud_file_name, 
		reference_point_cloud_file_name, 
		configuration_file_name,
		icp_output_folder;

	reading_point_cloud_file_name = std::string(argv[1]);

	reference_point_cloud_file_name = std::string(argv[2]);

	configuration_file_name = std::string(argv[3]);
	
	icp_output_folder = std::string(argv[4]);

	std::vector<point_3d> reading_point_cloud, reference_point_cloud;

	load_point_cloud_txt(reading_point_cloud_file_name, reading_point_cloud, false);

	load_point_cloud_txt(reference_point_cloud_file_name, reference_point_cloud, false);

	std::cout
		<< "file information:\n"

		<< "read point cloud:\n\t" << reading_point_cloud_file_name << " point size:" << reading_point_cloud.size() << "\n"

		<< "reference point cloud:\n\t" << reference_point_cloud_file_name << " point size:" << reference_point_cloud.size() << "\n"

		<< "configuration file for ICP algorithm:\n\t" << configuration_file_name << "\n"

		<< "output folder of this program:\n\t" << icp_output_folder << "\n";

	// fine regisration class
	cloud_registration m_cloud_registration;

	// final transformation for fine(icp) registration
	Eigen::Matrix4f fine_ret_mat;

	// transformed point cloud from reading point cloud, it will be saved locally
	std::vector<point_3d> fine_transformed_point_cloud;

	// implement iterative closest point algorithm that align reading point cloud to reference point cloud
	m_cloud_registration.fine_registration(reading_point_cloud, reference_point_cloud, configuration_file_name, fine_ret_mat);

	// matrix transforming reading point cloud to reference point cloud
	std::cout << "fine registration matrix is:\n" << fine_ret_mat << "\n";

	// apply for final tranformation on reading point cloud
	transform_points(reading_point_cloud, fine_ret_mat, fine_transformed_point_cloud);

	// save transformed reading point cloud with posefix "_transformed"
	save_points(fine_transformed_point_cloud, icp_output_folder + "/" + file_name_without_postfix(reading_point_cloud_file_name) + "_transformed.txt");

	cloud_viewer m_cloud_viewer("fine registration");

	std::vector<point_3d> reference_points;

	if (load_point_cloud_vtk(icp_output_folder + "/-reference-0.vtk", reference_points))
	{
		m_cloud_viewer.add_point_cloud_with_color(reference_points, 4.0, Eigen::Matrix4f::Identity(), 255, 0, 0);

		boost::thread update_reading_thread(load_file_to_display, icp_output_folder, &m_cloud_viewer, 0, 255, 0, 4.0, 200);
	}

	m_cloud_viewer.display();

	return 0;
}

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

#endif // FINE_REGISTRATION
