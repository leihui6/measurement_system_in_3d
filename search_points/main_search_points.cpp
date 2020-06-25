/*
*	This goal of this program is to search points in reference point cloud
*	besed on poits marked in reading point cloud.
*
*	Author: leihui.li#outlook.com
*	Date: 21/06/2020
*/

//#define SEARCH_POINTS
#ifdef SEARCH_POINTS

#include "cloud_io.h"
#include "cloud_search.h"
#include "common_use.h"

int main(int argc, char *argv[])
{
	if (argc < 3)
	{
		std::cerr
			<< "<reading_point_cloud>:\n\t" << "this program will search new points from this reading point cloud\n"
			<< "<output_folder>:\n\t" << "this program will load \"coarse_matrix\" and \"fine_matrix\" from this folder and search new points based on \"marked_points\" \n"
			<< "like: program_name <reading_point_cloud> <output_folder> \n";

		return -1;
	}

	std::string
		reading_point_cloud_file_name,
		output_folder;

	reading_point_cloud_file_name = std::string(argv[1]);

	output_folder = std::string(argv[2]);

	std::vector<point_3d> reading_point_cloud;

	load_point_cloud_txt(reading_point_cloud_file_name, reading_point_cloud, false);

	std::cout
		<< "file information:\n"

		<< "read point cloud:\n\t" << reading_point_cloud_file_name << " point size:" << reading_point_cloud.size() << "\n"

		<< "output folder of this program:\n\t" << output_folder << "\n";

	std::vector<Eigen::Matrix4f> matrix_vev;
	read_matrix(output_folder + "/coarse_matrix.txt", matrix_vev);
	read_matrix(output_folder + "/fine_matrix.txt", matrix_vev);

	Eigen::Matrix4f final_matrix;
	final_matrix = matrix_vev.back() * matrix_vev.front();

	transform_points(reading_point_cloud, final_matrix, reading_point_cloud);

	//save_points(reading_point_cloud, "test.txt");

	std::map<std::string, std::vector<point_3d>> marked_points_map;

	read_points(marked_points_map, output_folder + "/marked_points.txt");

	kd_tree kt(reading_point_cloud);

	std::map<std::string, std::vector<point_3d>>::iterator it;

	for (it = marked_points_map.begin(); it != marked_points_map.end(); it++)
	{
		std::vector<point_3d> & ptsv = it->second;

		kt.search_points_correspondence(ptsv);
	}

	export_marked_points(marked_points_map, output_folder + "/marked_points_searched.txt");

	std::cout << "marked points search from reading point cloud was exported to " << output_folder + "/marked_points_searched.txt" << std::endl;

	return 0;
}

#endif // !SEARCH_POINTS
