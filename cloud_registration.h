#ifndef CLOUD_REGISTRATION_H
#define CLOUD_REGISTRATION_H

#include "cloud_point.h"

class cloud_registration
{
public:
	cloud_registration();

	~cloud_registration();

	//! TODO, need to know more about this method
	/* 
	result matrix should be applied to points2
	points2->points1
	*/
	void coarse_registration(std::vector<point_3d> & points1, std::vector<point_3d> &points2, Eigen::Matrix4f & ret_mat);

	//! TODO, create a custom ICP algorithm
	/*
	result matrix should be applied to points2
	points2->points1
	*/
	void fine_registration(std::vector<point_3d> & points1, std::vector<point_3d> &points2, Eigen::Matrix4f & ret_mat);

	void get_final_transform_matrix(Eigen::Matrix4f & final_transform_matrix);

private:
	Eigen::Matrix4f coarse_transform_matrix;

	Eigen::Matrix4f fine_transform_matrix;
};

#endif // !CLOUD_REGISTRATION_H

