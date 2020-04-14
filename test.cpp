#include "cloud_io.h"
#include "osg_viewer.h"
#include "cloud_search.h"
#include "cloud_geometry.h"

int main()
{
	std::vector<point_3d> point_3d_vec;

	load_point_cloud_txt("line_point_cloud.txt", point_3d_vec);

	point_cloud m_point_cloud;

	m_point_cloud.load_points(point_3d_vec);

	line_func_3d line_func;

	fitting_line_3d_linear_least_squares(point_3d_vec, line_func);

	//kd_tree m_kd_tree(m_point_cloud);

	//std::vector<size_t> ret_index;

	//std::vector<float> out_dist_sqr;

	//m_kd_tree.search_neighbors_knn(10, point_3d_vec[171327], ret_index, out_dist_sqr);

	//std::vector<std::pair<size_t, float> > ret_matches;

	//m_kd_tree.search_neighbors_radius(10, point_3d_vec[171327], ret_matches);

	//osg_viewer m_osg_viewer;

	//m_osg_viewer.add_point_cloud(point_3d_vec);

	return 0;
}