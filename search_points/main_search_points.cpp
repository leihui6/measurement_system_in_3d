/*
*	This goal of this program is to search points in reference point cloud
*	besed on poits marked in reading point cloud.
*
*	Author: leihui.li#outlook.com
*	Date: 21/06/2020
*/

#define SEARCH_POINTS
#ifdef SEARCH_POINTS

#include "cloud_io.h"
#include "common_use.h"
#include "measurement.h"

// default parameters
// data/Armadillo_fine_registration_1.txt data/Armadillo_fine_registration_2.txt output

int main(int argc, char *argv[])
{
	if (argc < 4)
	{
		std::cerr
			<< "This program needs two point cloud as input, and a output folder\n"
			<< "you can enter more parameters like\n"
			<< "xxx.exe <reading_point_cloud.txt> <reference_point_cloud.txt> <output_folder> \n";

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

		<< "output folder of this program:\n\t" << output_folder << "\n";

	Eigen::Matrix4f coarse_matrix, fine_matrix, final_matrix;

	coarse_matrix = read_matrix(output_folder + "/coarse_matrix.txt");

	fine_matrix = read_matrix(output_folder + "/fine_matrix.txt");

	final_matrix = fine_matrix * coarse_matrix;

	transform_points(reading_point_cloud, final_matrix, reading_point_cloud);

	std::map<std::string, std::vector<point_3d>> marked_points_map;

	read_points(marked_points_map, output_folder + "/marked_points.txt");

	std::string points_1 = "line_0", points_2 = "point_0";

	measurement_value msv;

	measure(points_1, points_2, marked_points_map, msv);

	return 0;
}

#endif // !SEARCH_POINTS
