#ifndef CLOUD_POINT_H
#define CLOUD_POINT_H

#include <vector>

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


#endif // !CLOUD_POINT_H
