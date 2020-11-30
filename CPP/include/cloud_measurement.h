#ifndef CLOUD_MEASUREMENT
#define CLOUD_MEASUREMENT

#include "cloud_fitting.h"
#include "cloud_search.h"

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

	void analyse_defect(std::vector<point_3d>& scanned_points, std::vector<point_3d>& standard_model, std::vector<point_3d> & defect_points);

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
	void distance_scattered_points(
		std::vector<point_3d>& points_1,
		std::vector<point_3d>& points_2,
		float & distance);

	// point to line
	void distance_scattered_points(
		std::vector<point_3d>& points_1,
		std::vector<point_3d>& points_2, line_func_3d lf_2,
		float & distance);

	// point to plane
	void distance_scattered_points(
		std::vector<point_3d>& points_1,
		std::vector<point_3d>& points_2, plane_func_3d &plane_func_2,
		float & distance);

	// line to line
	void distance_scattered_points(
		std::vector<point_3d>& points_1, line_func_3d &lf_1, 
		std::vector<point_3d>& points_2, line_func_3d &lf_2, 
		float & distance);

	// line to line
	void distance_scattered_points(
		std::vector<point_3d>& points_1, line_func_3d &lf_1,
		std::vector<point_3d>& points_2, plane_func_3d &plane_func_2,
		float & distance);

	// plane to plane
	void distance_scattered_points(
		std::vector<point_3d>& points_1, plane_func_3d &plane_func_1, 
		std::vector<point_3d>& points_2, plane_func_3d &plane_func_2, 
		float & distance);
};

#endif // !CLOUD_MEASUREMENT


