#ifndef CLOUD_POINT_H
#define CLOUD_POINT_H

// Stardard library in c++ 11 
#include <vector>
#include <fstream>

// Eigen
#include <Eigen/Dense>

// CGAL
#include <CGAL/Simple_cartesian.h>

// OpenGR
#include <gr/algorithms/match4pcsBase.h>

// PointMatcher
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher/IO.h>

// OSG
#include <OSG/Vec3d>

struct point_3d 
{
	point_3d(const point_3d & p);

	point_3d();

	point_3d(float x, float y, float z);

	void set_xyz(float x, float y, float z);

	void set_nxyz(float nx, float ny, float nz);

	void set_rgb(float r, float g, float b);

	float x, y, z;

	float nx, ny, nz;

	float r, g, b;

	void to_eigen_vector4f(Eigen::Vector4f & vector4f_p);

	void do_transform(Eigen::Matrix4f & t, point_3d & p);

	friend std::ostream & operator << (std::ostream & os, const point_3d & p);
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

struct line_func_3d
{
	line_func_3d();

	float x, y, z;

	float n, m, l;

	void set_xyz(float x, float y, float z);

	void set_nml(float n, float m, float l);
};

struct plane_func
{
	plane_func();

	float a, b, c, d;
};

struct cylinder_func
{
	cylinder_func();

	line_func_3d m_line_func;

	float r;
};

point_3d to_point_3d(osg::Vec3d & p);

void convert_to_CGAL_points(std::vector<point_3d> & points, std::vector< CGAL::Simple_cartesian<float>::Point_3> &cgal_points);

void convert_to_original_points(std::vector< CGAL::Simple_cartesian<float>::Point_3> &cgal_points, std::vector<point_3d> & points);

void convert_to_openGR_points(std::vector<point_3d> & points, std::vector<gr::Point3D<float>> & opengr_points);

void convert_to_pointMatcher_points(std::vector<point_3d> & points, PointMatcher<float>::DataPoints & DP);

void transform_points(std::vector<point_3d>& points, Eigen::Matrix4f & t, std::vector<point_3d>& ret_points);

void pedalpoint_point_to_line(const point_3d & point, const line_func_3d & _line_func_3d, point_3d & pedalpoint);

void distance_points_to_line(const std::vector<point_3d>& points, const line_func_3d & _line_func_3d, std::vector<float>& points_dis_vec);

void distance_point_to_point(const point_3d & point_1, const point_3d & point_2, float & distance);

void save_points(const std::vector<point_3d>& points, const std::string & filename);

#endif // !CLOUD_POINT_H
