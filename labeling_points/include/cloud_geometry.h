#ifndef CLOUD_POINT_H
#define CLOUD_POINT_H

#include <windows.h>

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
#include <osg/Geometry>
#include <osg/Geode>

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

typedef point_3d point_3d;

struct line_func_3d
{
	line_func_3d();
	
	void set_xyz(float x, float y, float z);
	
	void set_nml(float n, float m, float l);
	
	float n, m, l;

	float x, y, z;
};

struct plane_func_3d
{
	plane_func_3d();

	void set_abcd(float a, float b, float c, float d);

	float a, b, c, d;
};

struct cylinder_func
{
	cylinder_func();

	line_func_3d m_line_func;

	float r;
};

// as nanoflann required
struct point_cloud
{
	std::vector<point_3d>  pts;

	// Must return the number of data points
	inline size_t kdtree_get_point_count() const
	{
		return pts.size();
	}


	// Returns the dim'th component of the idx'th point in the class:
	// Since this is inlined and the "dim" argument is typically an immediate value, the
	//  "if/else's" are actually solved at compile time.
	inline float kdtree_get_pt(const size_t idx, const size_t dim) const
	{
		if (dim == 0)
		{
			return pts[idx].x;
		}
		else if (dim == 1)
		{
			return pts[idx].y;
		}
		else
		{
			return pts[idx].z;
		}
	}

	// Optional bounding-box computation: return false to default to a standard bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX& /* bb */) const
	{
		return false;
	}

	void load_points(std::vector<point_3d> & points);
};

point_3d to_point_3d(osg::Vec3d & p);

void convert_to_CGAL_points(std::vector<point_3d> & points, std::vector< CGAL::Simple_cartesian<float>::Point_3> &cgal_points);

void convert_to_original_points(std::vector< CGAL::Simple_cartesian<float>::Point_3> &cgal_points, std::vector<point_3d> & points);

void convert_to_openGR_points(std::vector<point_3d> & points, std::vector<gr::Point3D<float>> & opengr_points);

void convert_to_pointMatcher_points(std::vector<point_3d> & points, PointMatcher<float>::DataPoints & DP);

void points_to_osg_structure(std::vector<point_3d>& points, osg::ref_ptr<osg::Vec3Array> coords, osg::ref_ptr<osg::Vec4Array> color, osg::ref_ptr<osg::Vec3Array> normals, float r = 0, float g = 0, float b = 0);

void points_to_geometry_node(std::vector<point_3d> & points, osg::ref_ptr<osg::Geometry> geometry, float r = 0, float g = 0, float b = 0);

// get minimal point and maximal point in a specific point set
void max_min_point_3d_vec(std::vector<point_3d> & points, point_3d & min_p, point_3d & max_p);

// get minimal value and maximal value in a specific array
void max_min_value_array(std::vector<float> & vec, float & min_value, float & max_value);

// calculate the min and max t in line function, min_max_t[0-2]: min_t; min_max[3-5]:max_t
void man_min_t_line_function(line_func_3d & line_func, point_3d & min_p, point_3d & max_p, std::vector<float>& min_t, std::vector<float> &max_t);

// calculate the appropriate t that could let point be closer to target point
void get_appropriate_t(line_func_3d & line_func, std::vector<float> t_vec, point_3d target_point, float & real_t);

// void get_distance_points_to_plane(std::vector<float> & points, plane_func_3d & plane_func, std::vector<float> & dis_to_plane);

void transform_points(std::vector<point_3d>& points, Eigen::Matrix4f & t, std::vector<point_3d>& ret_points);

// get a pedal point from a point to a line
void pedalpoint_point_to_line(const point_3d & point, const line_func_3d & _line_func_3d, point_3d & pedalpoint);

// get a pedal point from a point to a plane
void pedalpoint_point_to_plane(const point_3d & point, const plane_func_3d & plane_func, point_3d & pedalpoint);

void distance_points_to_line(const std::vector<point_3d>& points, const line_func_3d & _line_func_3d, std::vector<float>& points_dis_vec);

void distance_point_to_point(const point_3d & point_1, const point_3d & point_2, float & distance);

void save_points(const std::vector<point_3d>& points, const std::string & filename);

void make_points_ordered_by_distance(std::vector<point_3d>& points, std::vector<point_3d>& ordered_points);

void point_along_with_vector_within_dis(point_3d & point, Eigen::Vector3f & line_dir, point_3d & result_p1, point_3d & result_p2, float distance);

bool is_parallel_vector(const Eigen::Vector3f & v1, const Eigen::Vector3f & v2);

#endif // !CLOUD_POINT_H
