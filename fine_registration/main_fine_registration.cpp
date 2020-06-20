/*
*	The purpose of this program is to implement the fine registration
*
*	Author: Leihui Li
*	Date: 06/06/2020
*/

//#define FINE_REGISTRATION
#ifdef FINE_REGISTRATION

#include "cloud_io.h"
#include "cloud_viewer.h"
#include "cloud_fitting.h"
#include "cloud_registration.h"
#include "cloud_processing.h"
#include "common_use.h"

// default parameters:
// data/Armadillo_fine_registration_1.txt data/Armadillo_fine_registration_2.txt output/coarse_matrix.txt data/icp_configuration.yaml output

int main(int argc, char *argv[])
{
	if (argc < 5)
	{
		std::cerr
			//<< "please see help with \"--help\" \n "
			<< "This program needs two point cloud as input, a configruation file and a output folder\n"
			<< "you can enter more parameters like\n"
			<< "xxx.exe <reading_point_cloud.txt> <reference_point_cloud.txt> <initial_matrix.txt> <configruation_file.yaml> <icp_output> \n";

		return -1;
	}

	std::string
		reading_point_cloud_file_name, 
		reference_point_cloud_file_name, 
		initial_marix_file_name,
		configuration_file_name,
		icp_output_folder;

	reading_point_cloud_file_name = std::string(argv[1]);

	reference_point_cloud_file_name = std::string(argv[2]);

	initial_marix_file_name = std::string(argv[3]);

	configuration_file_name = std::string(argv[4]);
	
	icp_output_folder = std::string(argv[5]);

	std::vector<point_3d> reading_point_cloud, reference_point_cloud;

	// read reading point cloud from local file
	load_point_cloud_txt(reading_point_cloud_file_name, reading_point_cloud, false);

	Eigen::Matrix4f initial_matrix = read_matrix(initial_marix_file_name);

	transform_points(reading_point_cloud, initial_matrix, reading_point_cloud);

	// read reference point cloud from local file
	load_point_cloud_txt(reference_point_cloud_file_name, reference_point_cloud, false);

	std::cout
		<< "file information:\n"

		<< "read point cloud:\n\t" << reading_point_cloud_file_name << " point size:" << reading_point_cloud.size() << "\n"

		<< "reference point cloud:\n\t" << reference_point_cloud_file_name << " point size:" << reference_point_cloud.size() << "\n"
		
		<< "initial matrix of reading poin cloud:\n\t" << initial_marix_file_name << "\n"

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

	return 0;
}


#endif // FINE_REGISTRATION
