#include "cloud_io.h"
#include "cloud_viewer.h"
#include "cloud_search.h"
#include "cloud_geometry.h"
#include "cloud_registration.h"

int main()
{
	std::vector<point_3d> points_1_vec, points_2_vec;

	load_point_cloud_txt("data/model1.txt", points_1_vec);

	load_point_cloud_txt("data/model2.txt", points_2_vec);

	//cloud_registration m_cloud_registration;

	//m_cloud_registration.coarse_registration(points_1_vec, points_2_vec);

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

	cloud_viewer m_cloud_viewer("demo");

	m_cloud_viewer.add_point_cloud(points_1_vec);

	Eigen::Matrix4f test_transform;

	test_transform <<
		1, 0, 0, 10,
		0, 1, 0, 10,
		0, 0, 1, 10,
		0, 0, 0, 1;

	std::cout << test_transform << std::endl;

	m_cloud_viewer.add_point_cloud(points_1_vec, test_transform);

	m_cloud_viewer.display();

	return 0;
}