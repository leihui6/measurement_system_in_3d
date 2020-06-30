#ifndef CLOUD_FITTING_H
#define CLOUD_FITTING_H

#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>

#include "cloud_geometry.h"

#define HALF_PI 1.5707963267948966
#define TWO_PI 6.2831853071795862

typedef CGAL::Simple_cartesian<float> K;

class cloud_fitting
{
public:
	cloud_fitting();

	~cloud_fitting();

	void fitting_line_3d_linear_least_squares(std::vector<point_3d>& points, line_func_3d & line_func);

	void fitting_plane_3d_linear_least_squares(std::vector<point_3d>& points, plane_func_3d & plane_func);
	
	float fitting_cylinder_linear_least_squares(std::vector<point_3d>& points, cylinder_func & _cylinder_func);

// for cylinder
private:

	void preprocess(std::vector<point_3d>& points, Eigen::Vector3f & average);

	float computeSingleThreaded(Eigen::Vector3f & minPC, Eigen::Vector3f & minW, float& minRSqr);

	float computeMultiThreaded(Eigen::Vector3f & minPC, Eigen::Vector3f & minW, float & minRSqr);

	float G(Eigen::Vector3f const& W, Eigen::Vector3f & PC, float & rsqr);

	unsigned int m_num_theta_samples;
	unsigned int m_num_phi_samples;
	unsigned int m_num_threads;

	std::vector<Eigen::Vector3f> m_X;
	Eigen::Matrix<float, 6, 1> m_Mu;
	Eigen::Matrix<float, 3, 3> m_F0;
	Eigen::Matrix<float, 3, 6> m_F1;
	Eigen::Matrix<float, 6, 6> m_F2;

	//void get_drawable_cylinder_using_bottom_plane(std::vector<point_3d>& points, point_3d & bottom_center_p, plane_func_3d & plane_func, point_3d & center_p, float & radius, float & height);

	//void get_convex_hull_from_points(std::vector<point_3d>& points, std::vector<point_3d> & convex_hull_points);
};

#endif //CLOUD_FITTING_H