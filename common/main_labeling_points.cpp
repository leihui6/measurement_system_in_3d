/*
*	The purpose of this program is to load a standard 
*	and label the points representing the shapes to be deteced.
*
*	Author: leihui.li#outlook.com
*	Date: 06/06/2020
*/

#define LABELING_POINTS
#ifdef LABELING_POINTS

#include "cloud_io.h"
#include "cloud_viewer.h"
#include "interface_command.h"

int main(int argc, char *argv[])
{
	if (argc < 2) return 1;

	std::string point_cloud_file_name, output_folder_name,configuration_file_name;

	std::map<std::string, std::string> parameters;

	configuration_file_name = std::string(argv[1]);

	read_file_as_map(configuration_file_name, parameters);

	point_cloud_file_name = parameters["reference_point_cloud"];

	output_folder_name = parameters["output_folder"];

	std::cout
		<< "file information:\n"

		<< "standard point cloud:\n\t" << point_cloud_file_name << "\n"

		<< "output folder:\n\t" << output_folder_name << "\n";

	cloud_viewer m_cloud_viewer("labeling points(shapes) to be detected", parameters);

	interface_command ic(&m_cloud_viewer);

	ic.run();

	std::vector<point_3d> points_1_vec;

	load_point_cloud_txt(point_cloud_file_name, points_1_vec, false);

	m_cloud_viewer.add_point_cloud(points_1_vec);

	m_cloud_viewer.set_target_points(&points_1_vec);
	
	m_cloud_viewer.set_export_file_name(output_folder_name);

	m_cloud_viewer.display();

	return 0;
}

#endif // LABELING_POINTS
