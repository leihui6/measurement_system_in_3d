/*
*	The purpose of this program is to implement the coarse registration
*
*	Author: leihui.li#outlook.com
*	Date: 06/06/2020
*/

//#define coarse_REGISTRATION
#ifdef coarse_REGISTRATION

#include "cloud_io.h"
#include "cloud_registration.h"
#include "common_use.h"

int main(int argc, char *argv[])
{
	if (argc < 2) return 1;

	std::string configuration_file_name;
	configuration_file_name = std::string(argv[1]);
	std::map<std::string, std::string> parameters;
	read_file_as_map(configuration_file_name, parameters);

	std::string
		reading_point_cloud_file_name,
		reference_point_cloud_file_name,
		coarse_configuration,
		output_folder,
		coarse_registration_filename;

	reading_point_cloud_file_name = parameters["reading_data"];

	reference_point_cloud_file_name = parameters["reference_data"];

	coarse_configuration = parameters["4pcs_configuration"];

	output_folder = parameters["output_folder"];

	coarse_registration_filename = parameters["coarse_registration_filename"];

	std::vector<point_3d> reading_point_cloud, reference_point_cloud;

	load_point_cloud_txt(reading_point_cloud_file_name, reading_point_cloud, false);

	load_point_cloud_txt(reference_point_cloud_file_name, reference_point_cloud, false);

	std::cout
		<< "file information:\n"

		<< "read point cloud:\n\t" << reading_point_cloud_file_name << " point size:" << reading_point_cloud.size() << "\n"

		<< "reference point cloud:\n\t" << reference_point_cloud_file_name << " point size:" << reference_point_cloud.size() << "\n"

		<< "coarse_configuration:\n\t" << coarse_configuration << "\n"

		<< "final matrix of coarse registration export to:\n\t" << output_folder << "\n";

	// fine regisration class
	cloud_registration m_cloud_registration;

	// final transformation for fine(icp) registration
	Eigen::Matrix4f coarse_ret_mat;

	// transformed point cloud from reading point cloud, it will be saved locally
	std::vector<point_3d> coarse_transformed_point_cloud;

	// implement 4pcs algorithm that align reading point cloud to reference point cloud
	m_cloud_registration.coarse_registration(reading_point_cloud, reference_point_cloud, coarse_ret_mat, coarse_configuration);

	// matrix transforming reading point cloud to reference point cloud
	std::cout << "coarse registration matrix is:\n" << coarse_ret_mat << "\n";

	// save matrix to file
	save_matrix(coarse_ret_mat, output_folder + "/" + coarse_registration_filename);

	//system("pause");
	return 0;
}

#endif // coarse_REGISTRATION
