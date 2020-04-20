#ifndef CLOUD_POINT_H
#define CLOUD_POINT_H

// Stardard library in c++ 11 
#include <vector>

// Eigen
#include <Eigen/Dense>

// CGAL
#include <CGAL/Simple_cartesian.h>

// OpenGR
#include <gr/algorithms/match4pcsBase.h>

// PointMatcher
#include <pointmatcher/PointMatcher.h>

struct point_3d 
{
	point_3d();

	void set_xyz(float x, float y, float z);

	void set_nxyz(float nx, float ny, float nz);

	void set_rgb(float r, float g, float b);

	float x, y, z;

	float nx, ny, nz;

	float r, g, b;
};

// as nanoflann required
struct point_cloud
{
	std::vector<point_3d>  pts;

	// Must return the number of data points
	inline size_t kdtree_get_point_count() const { return pts.size(); }

	// Returns the dim'th component of the idx'th point in the class:
	// Since this is inlined and the "dim" argument is typically an immediate value, the
	//  "if/else's" are actually solved at compile time.
	inline float kdtree_get_pt(const size_t idx, const size_t dim) const
	{
		if (dim == 0) return pts[idx].x;
		else if (dim == 1) return pts[idx].y;
		else return pts[idx].z;
	}

	// Optional bounding-box computation: return false to default to a standard bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

	void load_points(std::vector<point_3d> & points)
	{
		this->pts = points;
	}
};

void convert_to_CGAL_points(std::vector<point_3d> & points, std::vector< CGAL::Simple_cartesian<float>::Point_3> &cgal_points);

void convert_to_openGR_points(std::vector<point_3d> & points, std::vector<gr::Point3D<float>> & opengr_points);

void convert_to_pointMatcher_points(std::vector<point_3d> & points, PointMatcher<float>::DataPoints & DP);

#endif // !CLOUD_POINT_H
