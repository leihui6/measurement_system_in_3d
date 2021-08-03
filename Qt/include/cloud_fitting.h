#ifndef CLOUD_FITTING_H
#define CLOUD_FITTING_H

#include <thread>

#include "cloud_geometry.h"

#define HALF_PI 1.5707963267948966
#define TWO_PI 6.2831853071795862

class cloud_fitting
{
public:
	cloud_fitting();

	~cloud_fitting();

	void fitting_line_3d_linear_least_squares(std::vector<point_3d>& points, line_func_3d & line_func);

	void fitting_plane_3d_linear_least_squares(std::vector<point_3d>& points, plane_func_3d & plane_func);
	
	float fitting_cylinder_linear_least_squares(std::vector<point_3d>& points, cylinder_func & _cylinder_func);

// only for fitting_cylinder_linear_least_squares usage inside
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
};

#endif //CLOUD_FITTING_H
