/*
*	The purpose of this program is to implement the fine registration
*
*	Author: Leihui Li
*	Date: 06/06/2020
*/

#define DISPLAY
#ifdef DISPLAY

#include "cloud_io.h"
#include "cloud_viewer.h"
#include "cloud_fitting.h"
#include "cloud_registration.h"
#include "cloud_processing.h"
#include "common_use.h"

// default parameters:
// data/Armadillo_fine_registration_1.txt data/Armadillo_fine_registration_2.txt output

int main(int argc, char *argv[])
{
	if (argc < 4)
	{
		std::cerr
			//<< "please see help with \"--help\" \n "
			<< "This program needs two point cloud as input, a configruation file and a output folder\n"
			<< "you can enter more parameters like\n"
			<< "xxx.exe <reading_point_cloud.txt> <reference_point_cloud.txt> <output> \n";

		return -1;
	}

	std::string
		reading_point_cloud_file_name,
		reference_point_cloud_file_name,
		output_folder;

	reading_point_cloud_file_name = std::string(argv[1]);

	reference_point_cloud_file_name = std::string(argv[2]);

	output_folder = std::string(argv[3]);

	std::vector<point_3d> reading_point_cloud, reference_point_cloud;

	// read reading point cloud from local file
	load_point_cloud_txt(reading_point_cloud_file_name, reading_point_cloud, false);

	// read reference point cloud from local file
	load_point_cloud_txt(reference_point_cloud_file_name, reference_point_cloud, false);

	std::cout
		<< "file information:\n"

		<< "read point cloud:\n\t" << reading_point_cloud_file_name << " point size:" << reading_point_cloud.size() << "\n"

		<< "reference point cloud:\n\t" << reference_point_cloud_file_name << " point size:" << reference_point_cloud.size() << "\n"

		<< "ioutput_folder:\n\t" << output_folder << "\n";
	
	std::vector<Eigen::Matrix4f> transformation_vec;

	// load coarse registration matrix
	std::string coarse_registration_matrix = output_folder + "/" + "coarse_matrix.txt";

	if (is_exist(coarse_registration_matrix))
	{
		transformation_vec.push_back(read_matrix(coarse_registration_matrix));
	}
	else
	{
		std::cerr << coarse_registration_matrix << " doesn't exist.\n";
	}

	// load fine registration matrics
	load_file_to_vec(output_folder, transformation_vec);

	// display all transformation repeatly
	cloud_viewer m_cloud_viewer("process of coarse and fine registration");

	m_cloud_viewer.add_point_cloud_with_color(reference_point_cloud, 4.0, Eigen::Matrix4f::Identity(), 255, 0, 0);

	boost::thread update_reading_thread(display_point_cloud_from_transformation_vec, m_cloud_viewer, reading_point_cloud, transformation_vec);

	m_cloud_viewer.display();

	return 0;
}


#endif // DISPLAY
