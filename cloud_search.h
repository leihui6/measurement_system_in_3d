#ifndef CLOUD_SEARCH_H
#define CLOUD_SEARCH_H

#include <nanoflann.hpp>
#include "cloud_geometry.h"

using namespace nanoflann;

typedef KDTreeSingleIndexAdaptor<
	L2_Simple_Adaptor<float, point_cloud>,
	point_cloud,
	3 /* dim */
> kd_tree_t;

class kd_tree
{
public:

	kd_tree(point_cloud & m_point_cloud);

	size_t search_neighbors_knn(size_t k, point_3d & p, std::vector<size_t> &ret_index, std::vector<float> &out_dist_sqr);

	size_t search_neighbors_radius(float search_radius, point_3d & p, std::vector<std::pair<size_t, float> > & ret_matches);

private:

	kd_tree_t m_kd_tree_t;

	void points_to_poincloud(std::vector<point_3d>& points, point_cloud & m_point_cloud);
};

#endif // !CLOUD_SEARCH_H
