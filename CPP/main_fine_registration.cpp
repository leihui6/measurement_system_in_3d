/*
*	The purpose of this program is to implement the fine registration
*
*	Author: leihui.li#outlook.com
*	Date: 06/06/2020
*/

//#define FINE_REGISTRATION
#ifdef FINE_REGISTRATION

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
		initial_marix_file_name,
		icp_configuration_file_name,
		output_folder;

	reading_point_cloud_file_name = parameters["reading_data"];
	reference_point_cloud_file_name = parameters["reference_data"];
	icp_configuration_file_name = parameters["icp_configuration"];
	output_folder = parameters["output_folder"];

	std::vector<point_3d> reading_point_cloud, reference_point_cloud;

	// read reading point cloud from local file
	load_point_cloud_txt(reading_point_cloud_file_name, reading_point_cloud, false);

	std::vector<Eigen::Matrix4f> initial_matrix;

	initial_marix_file_name = output_folder + "/coarse_matrix.txt";

	read_matrix(initial_marix_file_name, initial_matrix);

	transform_points(reading_point_cloud, initial_matrix.front(), reading_point_cloud);

	// read reference point cloud from local file
	load_point_cloud_txt(reference_point_cloud_file_name, reference_point_cloud, false);

	std::cout
		<< "file information:\n"

		<< "read point cloud:\n\t" << reading_point_cloud_file_name << " point size:" << reading_point_cloud.size() << "\n"

		<< "reference point cloud:\n\t" << reference_point_cloud_file_name << " point size:" << reference_point_cloud.size() << "\n"

		<< "initial matrix of reading poin cloud:\n\t" << initial_marix_file_name << "\n"

		<< "configuration file for ICP algorithm:\n\t" << icp_configuration_file_name << "\n"

		<< "output folder of this program:\n\t" << output_folder << "\n";

	// fine regisration class
	cloud_registration m_cloud_registration;

	// final transformation for fine(icp) registration
	Eigen::Matrix4f fine_ret_mat;

	// transformed point cloud from reading point cloud, it will be saved locally
	std::vector<point_3d> fine_transformed_point_cloud;

	// implement iterative closest point algorithm that align reading point cloud to reference point cloud
	m_cloud_registration.fine_registration(reading_point_cloud, reference_point_cloud, icp_configuration_file_name, fine_ret_mat);

	// matrix transforming reading point cloud to reference point cloud
	std::cout << "fine registration matrix is:\n" << fine_ret_mat << "\n";

	// save matrix to file
	//save_matrix(fine_ret_mat, output_folder + "/fine_matrix.txt");

	std::cout << "exported to " << output_folder << std::endl;

	//system("pause");
	return 0;
}


#endif // FINE_REGISTRATION
