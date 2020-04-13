#include "cloud_search.h"


kd_tree::kd_tree(point_cloud & m_point_cloud)
	:m_kd_tree_t(3, m_point_cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */))
{
	m_kd_tree_t.buildIndex();
}

size_t kd_tree::search_neighbors_knn(size_t k, point_3d & p, std::vector<size_t> &ret_index, std::vector<float> &out_dist_sqr)
{
	ret_index.resize(k);

	out_dist_sqr.resize(k);

	size_t num_results = m_kd_tree_t.knnSearch(&p.x, k, &ret_index[0], &out_dist_sqr[0]);

	// In case of less points in the tree than requested:
	ret_index.resize(num_results);
	out_dist_sqr.resize(num_results);

	//cout << "knnSearch(): num_results=" << num_results << "\n";
	//for (size_t i = 0; i < num_results; i++)
	//	cout << "idx[" << i << "]=" << ret_index[i] << " dist[" << i << "]=" << out_dist_sqr[i] << endl;
	//cout << "\n";

	return num_results;
}

size_t kd_tree::search_neighbors_radius(float search_radius, point_3d & p, std::vector<std::pair<size_t, float> > & ret_matches)
{
	nanoflann::SearchParams params;
	params.sorted = true;

	size_t nMatches = m_kd_tree_t.radiusSearch(&p.x, static_cast<float>(search_radius), ret_matches, params);

	return nMatches;
}

void kd_tree::points_to_poincloud(std::vector<point_3d>& points, point_cloud & m_point_cloud)
{
	m_point_cloud.pts = points;
}