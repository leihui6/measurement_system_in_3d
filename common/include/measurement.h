#ifndef MEASUREMENT
#define MEASUREMENT

#include "cloud_fitting.h"

struct measurement_value
{
	// distance based on geometry(function) // is_valid[0]
	float distance_geometry;

	// distance based on scattered points // is_valid[1]
	float distance_scattered;

	// angle in degree //// is_valid[2]
	float angle;

	// identify which one is valid for items above.
	bool is_valid[3];
};

class measurement
{
public:
	measurement();

	~measurement();

	virtual void measure(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_value & mv) = 0;

};

class point_to_point_measurement : measurement
{
public:
	virtual void measure(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_value & mv);
};

class point_to_line_measurement : measurement
{
public:
	virtual void measure(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_value & mv);
};

class point_to_plane_measurement : measurement
{
public:
	virtual void measure(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_value & mv);
};

class point_to_cylinder_measurement : measurement
{
public:
	virtual void measure(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_value & mv);
};

class line_to_line_measurement : measurement
{
public:
	virtual void measure(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_value & mv);
};

class line_to_plane_measurement : measurement
{
public:
	virtual void measure(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_value & mv);
};

class line_to_cylinder_measurement : measurement
{
public:
	virtual void measure(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_value & mv);
};

class plane_to_plane_measurement : measurement
{
public:
	virtual void measure(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_value & mv);
};

class plane_to_cylinder_measurement : measurement
{
public:
	virtual void measure(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_value & mv);
};

class cylinder_to_cylinder_measurement : measurement
{
public:
	virtual void measure(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_value & mv);
};

#endif // !MEASUREMENT


