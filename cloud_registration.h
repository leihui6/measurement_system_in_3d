#ifndef CLOUD_REGISTRATION_H
#define CLOUD_REGISTRATION_H

#include "cloud_point.h"

class cloud_registration
{
public:
	cloud_registration();

	~cloud_registration();

	// TODO, need to interact with upper calls
	void coarse_registration(std::vector<point_3d> & points1, std::vector<point_3d> &points2);
};

#endif // !CLOUD_REGISTRATION_H

