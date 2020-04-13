#include "cloud_io.h"
#include "osg_viewer.h"
#include "cloud_search.h"

int main()
{
	std::vector<point_3d> point_3d_vec;

	load_point_cloud_txt("model.asc", point_3d_vec);

	point_cloud m_point_cloud;

	m_point_cloud.load_points(point_3d_vec);

	kd_tree m_kd_tree(m_point_cloud);

	std::vector<size_t> ret_index;

	std::vector<float> out_dist_sqr;

	m_kd_tree.search_neighbors_knn(10, point_3d_vec[171327], ret_index, out_dist_sqr);

	std::vector<std::pair<size_t, float> > ret_matches;

	m_kd_tree.search_neighbors_radius(10, point_3d_vec[171327], ret_matches);

	//osg_viewer m_osg_viewer;

	//m_osg_viewer.add_point_cloud(point_3d_vec);

	return 0;
}