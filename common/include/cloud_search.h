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

	kd_tree(std::vector<point_3d> & points);
	
	kd_tree();

	~kd_tree();

	void load_points(std::vector<point_3d> & points);

	size_t search_neighbors_knn(size_t k, point_3d & p, std::vector<size_t> &ret_index, std::vector<float> &out_dist_sqr);

	size_t search_neighbors_radius(float search_radius, point_3d & p, std::vector<std::pair<size_t, float> > & ret_matches);

	size_t search_neighbors_radius(float search_radius, point_3d & p, std::vector<point_3d> & ret_index);

	void search_points_correspondence(std::vector<point_3d> & points, kd_tree & other_kd_tree, std::vector<point_3d> & other_points);

	void search_points_correspondence(std::vector<point_3d>& points, std::vector<point_3d>& found_points);

	void search_points_correspondence(std::vector<point_3d>& points);
	
	void get_point(size_t i, point_3d &p);

	point_3d get_point(size_t i);

private:

	point_cloud m_pc;

	kd_tree_t * m_kd_tree_t;

	void points_to_poincloud(std::vector<point_3d>& points, point_cloud & m_point_cloud);
};

#endif // !CLOUD_SEARCH_H
