/*
*	The purpose of this program is to implement the coarse registration
*
*	Author: Leihui Li
*	Date: 06/06/2020
*/

//#define coarse_REGISTRATION
#ifdef coarse_REGISTRATION

#include <filesystem>

#include "cloud_io.h"
#include "cloud_viewer.h"
#include "cloud_fitting.h"
#include "cloud_registration.h"
#include "cloud_processing.h"
#include "common_use.h"

// default parameters
// data/Armadillo_fine_registration_1.txt data/Armadillo_fine_registration_2.txt output

int main(int argc, char *argv[])
{
	if (argc < 4)
	{
		std::cerr
			//<< "please see help with \"--help\" \n "
			<< "This program needs two point cloud as input, a configruation file and a output folder\n"
			<< "you can enter more parameters like\n"
			<< "xxx.exe <reading_point_cloud.txt> <reference_point_cloud.txt> <output_folder>(final matrix of coarse registration)> \n";

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

	load_point_cloud_txt(reading_point_cloud_file_name, reading_point_cloud, false);

	load_point_cloud_txt(reference_point_cloud_file_name, reference_point_cloud, false);

	std::cout
		<< "file information:\n"

		<< "read point cloud:\n\t" << reading_point_cloud_file_name << " point size:" << reading_point_cloud.size() << "\n"

		<< "reference point cloud:\n\t" << reference_point_cloud_file_name << " point size:" << reference_point_cloud.size() << "\n"

		<< "final matrix of coarse registration export to:\n\t" << output_folder << "\n";

	// fine regisration class
	cloud_registration m_cloud_registration;

	// final transformation for fine(icp) registration
	Eigen::Matrix4f coarse_ret_mat;

	// transformed point cloud from reading point cloud, it will be saved locally
	std::vector<point_3d> coarse_transformed_point_cloud;

	// implement 4pcs algorithm that align reading point cloud to reference point cloud
	m_cloud_registration.coarse_registration(reading_point_cloud, reference_point_cloud, coarse_ret_mat);

	// matrix transforming reading point cloud to reference point cloud
	std::cout << "coarse registration matrix is:\n" << coarse_ret_mat << "\n";

	// save matrix to file
	save_matrix(coarse_ret_mat, output_folder + "/coarse_matrix.txt");

	std::cout << "exported to " << output_folder << std::endl;

	system("pause");

	return 0;
}

#endif // coarse_REGISTRATION
