/*
*	The purpose of this program is to implement the corase registration
*
*	Author: Leihui Li
*	Date: 06/06/2020
*/

//#define CORASE_REGISTRATION

#ifdef CORASE_REGISTRATION

#include <filesystem>

#include "cloud_io.h"
#include "cloud_viewer.h"
#include "cloud_fitting.h"
#include "cloud_registration.h"
#include "cloud_processing.h"
#include "common_use.h"

// default parameters
// data/Armadillo_fine_registration_1.txt data/Armadillo_fine_registration_2.txt data/Armadillo_fine_registration_1_corase_matrix.txt

int main(int argc, char *argv[])
{
	if (argc < 4)
	{
		std::cerr
			//<< "please see help with \"--help\" \n "
			<< "This program needs two point cloud as input, a configruation file and a output folder\n"
			<< "you can enter more parameters like\n"
			<< "xxx.exe <reading_point_cloud.txt> <reference_point_cloud.txt> <export_file_name(final matrix of corase registration)> \n";

		return -1;
	}

	std::string
		reading_point_cloud_file_name, 
		reference_point_cloud_file_name, 
		export_file_name;

	reading_point_cloud_file_name = std::string(argv[1]);

	reference_point_cloud_file_name = std::string(argv[2]);

	export_file_name = std::string(argv[3]);

	std::vector<point_3d> reading_point_cloud, reference_point_cloud;

	load_point_cloud_txt(reading_point_cloud_file_name, reading_point_cloud, false);

	load_point_cloud_txt(reference_point_cloud_file_name, reference_point_cloud, false);

	std::cout
		<< "file information:\n"

		<< "read point cloud:\n\t" << reading_point_cloud_file_name << " point size:" << reading_point_cloud.size() << "\n"

		<< "reference point cloud:\n\t" << reference_point_cloud_file_name << " point size:" << reference_point_cloud.size() << "\n"

		<< "final matrix of corase registration export to:\n\t" << export_file_name << "\n";

	// fine regisration class
	cloud_registration m_cloud_registration;

	// final transformation for fine(icp) registration
	Eigen::Matrix4f corase_ret_mat;

	// transformed point cloud from reading point cloud, it will be saved locally
	std::vector<point_3d> corase_transformed_point_cloud;

	// implement 4pcs algorithm that align reading point cloud to reference point cloud
	m_cloud_registration.coarse_registration(reading_point_cloud, reference_point_cloud, corase_ret_mat);

	// matrix transforming reading point cloud to reference point cloud
	std::cout << "corase registration matrix is:\n" << corase_ret_mat << "\n";

	// save matrix to file
	save_matrix(corase_ret_mat, export_file_name);

	std::cout << "exported to " << export_file_name << std::endl;

	system("pause");

	return 0;
}

#endif // CORASE_REGISTRATION
