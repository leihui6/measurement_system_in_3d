/*
*	The purpose of this program is to load a standard 
*	and label the points representing the shapes to be deteced.
*
*	Author: Leihui Li
*	Date: 06/06/2020
*/

#define LABELING_POINTS
#ifdef LABELING_POINTS

#include "cloud_io.h"
#include "cloud_viewer.h"
#include "cloud_search.h"
#include "cloud_fitting.h"
#include "cloud_registration.h"
#include "cloud_processing.h"
#include "interface_command.h"

// default parameters
// points_withnxyz_k_part.txt output

int main(int argc, char *argv[])
{
	if (argc < 3)
	{
		std::cerr
			//<< " please see help with \"--help\" " << std::endl;
			<< "This program needs two parameters including a standard point cloud and a output folder.\n"
			<< "like: program_name <point_cloud> <output_folder>\n";
		return -1;
	}

	std::string point_cloud_file_name, output_folder_name;

	point_cloud_file_name = std::string(argv[1]);

	output_folder_name = std::string(argv[2]);

	std::cout
		<< "file information:\n"

		<< "standard point cloud:\n\t" << point_cloud_file_name << "\n"

		<< "output folder:\n\t" << output_folder_name << "\n";

	cloud_viewer m_cloud_viewer("labeling points(shapes) to be detected");

	interface_command ic(&m_cloud_viewer);

	ic.run();

	std::vector<point_3d> points_1_vec;

	load_point_cloud_txt(point_cloud_file_name, points_1_vec, false);

	m_cloud_viewer.add_point_cloud(points_1_vec, 10);

	m_cloud_viewer.set_the_target_points(&points_1_vec);
	
	m_cloud_viewer.set_export_file_name(output_folder_name);

	m_cloud_viewer.display();

	return 0;
}

#endif // LABELING_POINTS
