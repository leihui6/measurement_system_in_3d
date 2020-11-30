#ifndef CLOUD_PROCESSING_H
#define CLOUD_PROCESSING_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/remove_outliers.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/wlop_simplify_and_regularize_point_set.h>

#include "cloud_geometry.h"

#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif

class cloud_processing
{
public:
	cloud_processing();
	
	~cloud_processing();

	void get_average_spacing(std::vector<point_3d> & points, float & average_spacing, const size_t nb_neighbors = 6);

	//void estimating_normals_with_radius(std::vector<point_3d>& points, float radius);

	void estimate_normals_with_k(std::vector<point_3d>& points, size_t k);

	void filter_remove_outliers(std::vector<point_3d>& points, size_t k, float threshold_distance);

	void filter_simplify_grid(std::vector<point_3d>& points, float cell_size);

	void filter_simplify_wlop(std::vector<point_3d>& points, float retain_percentage, float neighbor_radius);

private:

};

#endif // !CLOUD_PROCESSING_H

