#ifndef CLOUD_MEASUREMENT
#define CLOUD_MEASUREMENT

#include "cloud_fitting.h"

#define INVALIDVALUE -1.0f

struct measurement_value
{
	measurement_value() :
		distance_geometrical(INVALIDVALUE),
		distance_scattered(INVALIDVALUE),
		angle(INVALIDVALUE) {}
	// distance based on geometry(function)
	float distance_geometrical;

	// distance based on scattered points
	float distance_scattered;

	// angle in degree 
	float angle;
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
	void distance_scattered_points(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, float & distance);
};

#endif // !CLOUD_MEASUREMENT


