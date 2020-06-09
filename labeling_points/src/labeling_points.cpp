/*
*	The purpose of this program is to load a standard 
*	and label the points representing the shapes to be deteced.
*
*	Author: Leihui Li
*	Date: 06/06/2020
*/

#include "cloud_io.h"
#include "cloud_viewer.h"
#include "cloud_search.h"
#include "cloud_fitting.h"
#include "cloud_registration.h"
#include "cloud_processing.h"
#include "interface_command.h"

//#define TEST_NORMAL
//#define TEST_REGISTRATION

int main(int argc, char *argv[])
{
	if (argc < 2)
	{
		std::cerr << " please see help with \"--help\" "<< std::endl;
		return -1;
	}

	interface_command ic;

	ic.run();

	std::string standard_point_cloud_filename;

	standard_point_cloud_filename = std::string(argv[1]);

	std::vector<point_3d> points_1_vec, points_2_vec;

	load_point_cloud_txt(standard_point_cloud_filename, points_1_vec);

	cloud_viewer m_cloud_viewer("labeling points(shapes) to be detected");

	m_cloud_viewer.add_point_cloud(points_1_vec, 10);

	m_cloud_viewer.set_the_target_points(points_1_vec);

	m_cloud_viewer.set_the_interface_command(&ic);

	m_cloud_viewer.display();

	//load_point_cloud_txt("data/HeatShield00_01.txt", points_2_vec);

	//cloud_processing m_cloud_processing;

#ifdef TEST_AVERAGE_SPACING
	float average_spacing_1, average_spacing_2;
	m_cloud_processing.get_average_spacing(points_1_vec, average_spacing_1);
	m_cloud_processing.get_average_spacing(points_2_vec, average_spacing_2);
	std::cout << "average_spacing_1=" << average_spacing_1 << std::endl;
	std::cout << "average_spacing_2=" << average_spacing_2 << std::endl;
#endif // TEST_AVERAGE_SPACING

#ifdef TEST_NORMAL
	// estimating the normals of points
	//m_cloud_processing.estimating_normals_with_radius(points_1_vec, 1.0);

	//m_cloud_processing.estimate_normals_with_k(points_1_vec, 18);

	//m_cloud_processing.filter_remove_outliers(points_1_vec, 18, 0.5);

	m_cloud_processing.filter_simplify_grid(points_2_vec, 0.25);

	//m_cloud_processing.filter_simplify_wlop(points_2_vec, 8.0, 1.0);

#endif // TEST_NORMAL

#ifdef TEST_REGISTRATION
	// registration
	cloud_registration m_cloud_registration;

	Eigen::Matrix4f coarse_ret_mat, fine_ret_mat, final_registration_matrix;

	std::vector<point_3d> points_2_vec_transformed_coarse, points_2_vec_transformed_fine;

	// coarse registration
	m_cloud_registration.coarse_registration(points_1_vec, points_2_vec, coarse_ret_mat);

	std::cout << "coarse registration matrix is:" << std::endl << coarse_ret_mat << std::endl;

	transform_points(points_2_vec, coarse_ret_mat, points_2_vec_transformed_coarse);

	save_points(points_2_vec_transformed_coarse, "data/HeatShield01_tranformed_coarse.txt");

	// fine registration
	m_cloud_registration.fine_registration(points_1_vec, points_2_vec_transformed_coarse, fine_ret_mat);

	std::cout << "fine registration matrix is:" << std::endl << fine_ret_mat << std::endl;

	transform_points(points_2_vec_transformed_coarse, fine_ret_mat, points_2_vec_transformed_fine);

	save_points(points_2_vec_transformed_fine, "data/HeatShield01_tranformed_fine.txt");

	// get the final registration matrix
	m_cloud_registration.get_final_transform_matrix(final_registration_matrix);

	std::cout << final_registration_matrix << std::endl;

#endif // TEST_REGISTRATION

	//point_cloud m_point_cloud;

	//m_point_cloud.load_points(point_3d_vec);

	//line_func_3d line_func;

	//fitting_line_3d_linear_least_squares(point_3d_vec, line_func);

	//kd_tree m_kd_tree(m_point_cloud);

	//std::vector<size_t> ret_index;

	//std::vector<float> out_dist_sqr;

	//m_kd_tree.search_neighbors_knn(10, point_3d_vec[171327], ret_index, out_dist_sqr);

	//std::vector<std::pair<size_t, float> > ret_matches;

	//m_kd_tree.search_neighbors_radius(10, point_3d_vec[171327], ret_matches);

	//cloud_viewer m_cloud_viewer("demo");

	//m_cloud_viewer.add_point_cloud(points_1_vec);

	//m_cloud_viewer.set_the_target_points(points_1_vec);

	//std::vector <point_3d > picked_points;
	//
	//m_cloud_viewer.get_picked_points(picked_points);

	//m_cloud_viewer.add_test_points();
	//Eigen::Matrix4f test_transform;
	//test_transform <<
	//	1.000000,0.000000,0.000000,-1.000000,
	//	0.000000,0.866000,-0.500000,-2.000000,
	//	0.000000,0.500000,0.866000,3.000000,
	//	0.000000,0.000000,0.000000,1.000000;
	//std::cout << test_transform << std::endl;
	//m_cloud_viewer.add_point_cloud(points_1_vec, 4.0, test_transform);
	//m_cloud_viewer.add_model("data/cow.osg");

	//m_cloud_viewer.display();

	return 0;
}