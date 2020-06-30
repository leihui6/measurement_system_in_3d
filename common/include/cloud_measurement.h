#ifndef CLOUD_MEASUREMENT
#define CLOUD_MEASUREMENT

#include "cloud_fitting.h"

struct measurement_value
{
	// distance based on geometry(function)
	// is_valid[0]
	float distance_geometry;

	// distance based on scattered points
	// is_valid[1]
	// [0] [min]
	// [1] [max]
	float distance_scattered[2];

	// angle in degree 
	// is_valid[2]
	float angle;

	// identify which one is valid for items above.
	bool is_valid[3];
};

class cloud_measurement
{
public:
	cloud_measurement();

	~cloud_measurement();

	void measure(std::multimap<std::string, std::string> measurement_pairs_map, std::map<std::string, std::vector<point_3d>>& _m, std::vector<measurement_value> & mv_vec);

private:

	void measure(std::string & points_1_name, std::string & points_2_name, std::map<std::string, std::vector<point_3d>>& _m, measurement_value & mv);

	void analyze_points(std::vector<size_t> & order_1, std::vector<size_t> & order_2, std::vector<point_3d> points_1, std::vector<point_3d> points_2, measurement_value & mv);

	cloud_fitting cf;

private:
	// the first points belongs to the type of point
	void calculate_point_to_point(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_value & mv);
	void calculate_point_to_line(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_value & mv);
	void calculate_point_to_plane(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_value & mv);
	void calculate_point_to_cylinder(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_value & mv);

	// the first points belongs to the type of line
	void calculate_line_to_line(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_value & mv);
	void calculate_line_to_plane(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_value & mv);
	void calculate_line_to_cylinder(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_value & mv);

	// the first points belongs to the type of plane
	void calculate_plane_to_plane(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_value & mv);
	void calculate_plane_to_cylinder(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_value & mv);

	// the first points belongs to the type of cylinder
	void calculate_cylinder_to_cylinder(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_value & mv);

private:
	void distance_scattered_points(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, float & min_distance, float & max_distance);
};

#endif // !CLOUD_MEASUREMENT


