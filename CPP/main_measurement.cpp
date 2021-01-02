/*

*/

#define MEASUREMENT
#ifdef MEASUREMENT

#include "common_use.h"
#include "cloud_measurement.h"

int main(int argc, char *argv[])
{
	if (argc < 2) return 1;

	std::string configuration_file_name;
	configuration_file_name = std::string(argv[1]);
	std::map<std::string, std::string> parameters;
	read_file_as_map(configuration_file_name, parameters);

	std::string
		measurement_pairs,
		output_folder,
		searched_marked_points_file_name;

	measurement_pairs = parameters["measurement_pairs"];

	output_folder = parameters["output_folder"];

	std::cout
		<< "file information:\n"

		<< "measurement configuration file_name:\n\t" << measurement_pairs << "\n"

		<< "output folder:\n\t" << output_folder << "\n";

	// load marked points
	std::map<std::string, std::vector<point_3d>> marked_points_map;
	read_points(marked_points_map, output_folder + "/marked_points_searched.txt");

	// load measure pairs
	std::multimap<std::string, std::string> measurement_pairs_map;
	read_file_as_map(measurement_pairs, measurement_pairs_map);

	// load reading and reference point cloud
	std::string
		reading_data_name = parameters["reading_data"],
		reference_data_name = parameters["reference_data"];
	std::vector<point_3d> reading_data, reference_data;
	load_point_cloud_txt(reading_data_name, reading_data);
	load_point_cloud_txt(reference_data_name, reference_data);
	
	std::vector<Eigen::Matrix4f> matrix_vev;
	read_matrix(output_folder + "/coarse_matrix.txt", matrix_vev);
	read_matrix(output_folder + "/fine_matrix.txt", matrix_vev);
	Eigen::Matrix4f final_matrix;
	final_matrix = matrix_vev.back() * matrix_vev.front();
	transform_points(reading_data, final_matrix, reading_data);

	std::cout << "load:" << " reading data:" << reading_data.size() << "reference_data:" << reference_data.size() << std::endl;

	// begin to measurement including dimensional inspection and defect analysis
	// 1. dimensional inspection
	cloud_measurement cm;
	std::vector<measurement_value> mv_vec;
	cm.measure(measurement_pairs_map, marked_points_map, mv_vec);
	export_measured_data(measurement_pairs_map, mv_vec, output_folder + "/measurement_result.txt");

	// 2. defect analysis
	std::vector<point_3d> defect_points;
	cm.analyse_defect(reading_data, reference_data, defect_points);
	save_points(defect_points, output_folder + "/defect_points.txt");

	//system("pause");
	return 0;
}

#endif

