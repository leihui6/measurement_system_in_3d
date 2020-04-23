#include "cloud_processing.h"



cloud_processing::cloud_processing()
{
}


cloud_processing::~cloud_processing()
{
}

void cloud_processing::get_average_spacing(std::vector<point_3d>& points, float & average_spacing, const size_t nb_neighbors)
{
	std::vector<CGAL::Simple_cartesian<float>::Point_3> cgal_points;

	convert_to_CGAL_points(points, cgal_points);

	average_spacing 
		= CGAL::compute_average_spacing<Concurrency_tag>(cgal_points, nb_neighbors);
}

void cloud_processing::estimating_normals_with_radius(std::vector<point_3d>& points, float radius)
{
	typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;

	typedef Kernel::Vector_3 Vector;

	typedef std::pair<Kernel::Point_3, Vector> PointVectorPair;

	std::vector<CGAL::Simple_cartesian<float>::Point_3> cgal_points;

	std::list<PointVectorPair> points_with_nxyz;

	for (size_t i = 0; i < points.size(); ++i)
	{
		points_with_nxyz.push_back(std::pair<Kernel::Point_3, Vector>
			({ points[i].x, points[i].y, points[i].z }, {}));
	}

	CGAL::pca_estimate_normals<Concurrency_tag>
		(points_with_nxyz,
			0, // when using a neighborhood radius, K=0 means no limit on the number of neighbors returns
			CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>()).
			normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>()).neighbor_radius(radius));

	size_t i = 0;

	for (std::list<PointVectorPair>::iterator it = points_with_nxyz.begin(); it != points_with_nxyz.end(); ++it)
	{
		points[i].set_nxyz(it->second[0], it->second[1], it->second[2]);
		
		++i;
	}

	//save_points(points, "data/points_withnxyz.txt");
}

void cloud_processing::estimating_normals_with_k(std::vector<point_3d>& points, size_t k)
{
	typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;

	typedef Kernel::Vector_3 Vector;

	typedef std::pair<Kernel::Point_3, Vector> PointVectorPair;

	std::vector<CGAL::Simple_cartesian<float>::Point_3> cgal_points;

	std::list<PointVectorPair> points_with_nxyz;

	for (size_t i = 0; i < points.size(); ++i)
	{
		points_with_nxyz.push_back(std::pair<Kernel::Point_3, Vector>
			({ points[i].x, points[i].y, points[i].z }, {}));
	}

	CGAL::pca_estimate_normals<Concurrency_tag>
		(points_with_nxyz, k,
			CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>()).
			normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>()));

	size_t i = 0;

	for (std::list<PointVectorPair>::iterator it = points_with_nxyz.begin(); it != points_with_nxyz.end(); ++it)
	{
		points[i].set_nxyz(it->second[0], it->second[1], it->second[2]);

		++i;
	}

	//save_points(points, "data/points_withnxyz.txt");
}
