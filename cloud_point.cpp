#include "cloud_point.h"

point_3d::point_3d()
	:x(0), y(0), z(0),
	nx(0), ny(0), nz(0),
	r(0), g(0), b(0)
{
}

void point_3d::set_xyz(float x, float y, float z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

void point_3d::set_nxyz(float nx, float ny, float nz)
{
	this->nx = nx;
	this->ny = ny;
	this->nz = nz;
}

void point_3d::set_rgb(float r, float g, float b)
{
	this->r = r;
	this->g = g;
	this->b = b;
}
