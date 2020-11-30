#include "cloud_search.h"

kd_tree::kd_tree(std::vector<point_3d> & points)
{
	m_pc.load_points(points);

	m_kd_tree_t = new kd_tree_t(3, m_pc, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));

	m_kd_tree_t->buildIndex();
}

kd_tree::kd_tree()
{
}


kd_tree::~kd_tree()
{
	if (m_kd_tree_t)
	{
		delete m_kd_tree_t;
	}
}

void kd_tree::load_points(std::vector<point_3d>& points)
{
	m_pc.load_points(points);

	m_kd_tree_t = new kd_tree_t(3, m_pc, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));

	m_kd_tree_t->buildIndex();
}

size_t kd_tree::search_neighbors_knn(size_t k, point_3d & p, std::vector<size_t> &ret_index, std::vector<float> &out_dist_sqr)
{
	ret_index.resize(k);

	out_dist_sqr.resize(k);

	size_t num_results = m_kd_tree_t->knnSearch(&p.x, k, &ret_index[0], &out_dist_sqr[0]);

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

	size_t nMatches = m_kd_tree_t->radiusSearch(&p.x, static_cast<float>(search_radius), ret_matches, params);

	return nMatches;
}

size_t kd_tree::search_neighbors_radius(float search_radius, point_3d & p, std::vector<point_3d> & ret_points)
{
	nanoflann::SearchParams params;
	params.sorted = true;

	std::vector<std::pair<size_t, float> > ret_matches;
	size_t nMatches = m_kd_tree_t->radiusSearch(&p.x, static_cast<float>(search_radius), ret_matches, params);

	for (size_t i = 0; i < ret_matches.size(); ++i)
	{
		ret_points.push_back(this->m_pc.pts[ret_matches[i].first]);
	}

	return ret_points.size();
}

void kd_tree::search_points_correspondence(std::vector<point_3d>& points, kd_tree & other_kd_tree, std::vector<point_3d>& other_points)
{
	for (size_t i = 0; i < points.size(); ++i)
	{
		std::vector<size_t> ret_index;
		std::vector<float> ret_dis;
		other_kd_tree.search_neighbors_knn(1, points[i], ret_index, ret_dis);

		if (ret_index.size() == 1)
		{
			other_points.push_back(get_point(ret_index.front()));
		}
		else if(ret_index.empty())
		{
			// error
			std::cerr << __LINE__ << "search_points_correspondence()" << points[i] << " has no result in thie cloud." << std::endl;
		}
	}
}

void kd_tree::search_points_correspondence(std::vector<point_3d>& points, std::set <size_t> & correspondences_index, float dis_threshold)
{
	std::vector<bool> visited_index(this->m_pc.pts.size(), false);

	for (size_t i = 0; i < points.size(); ++i)
	{
		std::vector<size_t> ret_index;

		std::vector<float> ret_dis;

		std::vector<std::pair<size_t, float>> ret_matches;

		this->search_neighbors_radius(dis_threshold, points[i], ret_matches);

		if (ret_matches.size() > 1)
		{
			for (size_t j = 0; j < ret_matches.size(); ++j)
			{
				size_t tar_j = ret_matches[j].first;

				if (visited_index[tar_j] == false)
				{
					correspondences_index.insert(tar_j);

					visited_index[tar_j] = true;
				}
			}
		}
		else if (ret_index.empty())
		{
			// error
			// std::cerr << __LINE__ << "search_points_correspondence()" << points[i] << " has no result in thie cloud." << std::endl;
		}
	}
}

void kd_tree::search_points_correspondence(std::vector<point_3d>& points, std::vector<point_3d>& correspondences_points)
{
	for (size_t i = 0; i < points.size(); ++i)
	{
		std::vector<size_t> ret_index;

		std::vector<float> ret_dis;

		this->search_neighbors_knn(1, points[i], ret_index, ret_dis);

		if (!ret_index.empty())
		{
			correspondences_points.push_back(get_point(ret_index.front()));
		}
		else if (ret_index.empty())
		{
			// error
			std::cerr << __LINE__ << "search_points_correspondence()" << points[i] << " has no result in thie cloud." << std::endl;
		}
	}
}

void kd_tree::get_point(size_t i, point_3d & p)
{
	p = this->m_kd_tree_t->dataset.pts[i];
}

point_3d kd_tree::get_point(size_t i)
{
	return this->m_kd_tree_t->dataset.pts[i];
}

void kd_tree::points_to_poincloud(std::vector<point_3d>& points, point_cloud & m_point_cloud)
{
	m_point_cloud.pts = points;
}