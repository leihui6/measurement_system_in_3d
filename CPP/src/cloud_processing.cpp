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

//void cloud_processing::estimating_normals_with_radius(std::vector<point_3d>& points, float radius)
//{
//	typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
//
//	typedef Kernel::Vector_3 Vector;
//
//	typedef std::pair<Kernel::Point_3, Vector> PointVectorPair;
//
//	typedef std::vector<Point_with_normal> PointList;
//
//	std::vector<CGAL::Simple_cartesian<float>::Point_3> cgal_points;
//
//	std::list<PointVectorPair> points_with_nxyz;
//
//	for (size_t i = 0; i < points.size(); ++i)
//	{
//		points_with_nxyz.push_back(std::pair<Kernel::Point_3, Vector>
//			({ points[i].x, points[i].y, points[i].z }, {}));
//	}
//
//	CGAL::pca_estimate_normals<Concurrency_tag>
//		(points_with_nxyz,
//			2, // when using a neighborhood radius, K=0 means no limit on the number of neighbors returns
//			CGAL::parameters::normal_map(CGAL::make_normal_of_point_with_normal_map(PointList::value_type())));
//
//	size_t i = 0;
//
//	for (std::list<PointVectorPair>::iterator it = points_with_nxyz.begin(); it != points_with_nxyz.end(); ++it)
//	{
//		points[i].set_nxyz(it->second[0], it->second[1], it->second[2]);
//		
//		++i;
//	}
//
//	save_points(points, "data/points_withnxyz_radius.txt");
//}

void cloud_processing::estimate_normals_with_k(std::vector<point_3d>& points, size_t k)
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

	CGAL::pca_estimate_normals<Concurrency_tag>(points_with_nxyz, k, 
		CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>()).
		normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>()));

	std::list<PointVectorPair>::iterator unoriented_points_begin =
		CGAL::mst_orient_normals(points_with_nxyz, k,
			CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>()).
			normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>()));

	// optional
	//points_with_nxyz.erase(unoriented_points_begin, points_with_nxyz.end());

	size_t i = 0;

	for (std::list<PointVectorPair>::iterator it = points_with_nxyz.begin(); it != points_with_nxyz.end(); ++it)
	{
		points[i].set_nxyz(it->second[0], it->second[1], it->second[2]);

		++i;
	}

	save_points(points, "data/points_withnxyz_k.txt");
}

void cloud_processing::filter_remove_outliers(std::vector<point_3d>& points, size_t k, float threshold_distance)
{
	std::vector<CGAL::Simple_cartesian<float>::Point_3> cgal_points;

	convert_to_CGAL_points(points, cgal_points);

	std::vector<CGAL::Simple_cartesian<float>::Point_3>::iterator first_to_remove
		= CGAL::remove_outliers
		(cgal_points,
			k,
			CGAL::parameters::threshold_percent(100.).threshold_distance(threshold_distance));
	
	std::cerr << (100. * std::distance(first_to_remove, cgal_points.end()) / (double)(cgal_points.size()))
		<< "% of the points are considered outliers when using a distance threshold of " << std::endl;

	cgal_points.erase(first_to_remove, cgal_points.end());

	convert_to_original_points(cgal_points, points);

	save_points(points, "data/removed_outliers.txt");
}

void cloud_processing::filter_simplify_grid(std::vector<point_3d>& points, float cell_size)
{
	std::vector<CGAL::Simple_cartesian<float>::Point_3> cgal_points;

	convert_to_CGAL_points(points, cgal_points);

	std::vector<CGAL::Simple_cartesian<float>::Point_3>::iterator first_to_remove
		= CGAL::grid_simplify_point_set(cgal_points, cell_size);

	cgal_points.erase(first_to_remove, cgal_points.end());

	// test
	std::cout << "original point size=" << points.size() << " " << " after filter, point size=" << cgal_points.size() << std::endl;

	convert_to_original_points(cgal_points, points);

	// test
	save_points(points, "data/filter_grid_simplify.txt");
}

void cloud_processing::filter_simplify_wlop(std::vector<point_3d>& points, float retain_percentage, float neighbor_radius)
{
	std::vector<CGAL::Simple_cartesian<float>::Point_3> cgal_points, output;

	convert_to_CGAL_points(points, cgal_points);

	CGAL::wlop_simplify_and_regularize_point_set<Concurrency_tag>
		(cgal_points, 
			std::back_inserter(output),
			CGAL::parameters::select_percentage(retain_percentage).
			neighbor_radius(neighbor_radius));

	convert_to_original_points(output, points);

	save_points(points, "data/filter_simplify_wlop.txt");
}
