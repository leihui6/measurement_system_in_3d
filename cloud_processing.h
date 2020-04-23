#ifndef CLOUD_PROCESSING_H
#define CLOUD_PROCESSING_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/pca_estimate_normals.h>

#include "cloud_point.h"

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

	void estimating_normals_with_radius(std::vector<point_3d>& points, float radius);

	void estimating_normals_with_k(std::vector<point_3d>& points, size_t k);

private:

};

#endif // !CLOUD_PROCESSING_H

