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

extern void measure(std::string & points_1_name, std::string & points_2_name, std::map<std::string, std::vector<point_3d>> & _m, measurement_value & mv);

extern void analyze_points(std::vector<size_t> & order_1, std::vector<size_t> & order_2, std::vector<point_3d> points_1, std::vector<point_3d> points_1, measurement_value & mv);

#endif // !MEASUREMENT


