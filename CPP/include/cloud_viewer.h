#ifndef cloud_viewer_H
#define cloud_viewer_H

#include <windows.h>
//#include <WinDef.h>
//#include <WinGDI.h>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osg/MatrixTransform>
#include <osgGA/GUIEventHandler>
#include <osg/Point>
#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/LineWidth>
#include <osgUtil/SceneView>
#include <osgUtil/IntersectionVisitor>
#include <osgViewer/Renderer>
#include <osgDB/ReadFile>
#include <osgUtil/Optimizer> 
#include <osgUtil/DelaunayTriangulator> 
#include <osgGA/TrackballManipulator>
#include <osg/ComputeBoundsVisitor>

#include "common_use.h"
#include "interface_command.h"
#include "cloud_fitting.h"
#include "cloud_geometry.h"
#include "cloud_search.h"
#include "cloud_pickhandler.h"

class PickHandler;

//class cloud_viewer;

class interface_command;

struct viewer_parameters
{
	viewer_parameters() :
		picking_range(0.1),
		background_color(0, 0, 0, 1),
		point_size(4.0),
		point_color(1, 1, 1, 1),
		picked_point_size(10.0),
		is_auto_pick(true)
	{
		auto_pick_line_threshold = 0.1;
		auto_pick_plane_threshold = 0.1;
		auto_pick_point_threshold = 0.1;
		fitting_line_width = 3.0;
		fitting_plane_transparency = 0.2;
	}
	// default:0.1
	float picking_range;
	// default: 0 0 0 1
	osg::Vec4 background_color;
	// default: 4.0
	float point_size;
	// default: 
	osg::Vec4 point_color;
	float picked_point_size;
	bool is_auto_pick;
	// threshold used for collect more points representing a line
	float auto_pick_line_threshold;
	float auto_pick_plane_threshold;
	float auto_pick_point_threshold;

	float fitting_line_width;
	float fitting_plane_transparency;

	void set_background_color(osg::Vec4 c)
	{
		background_color.set(c[0] / 255, c[1] / 255, c[2] / 255, c[3] / 255);
	}
};

class cloud_viewer
{
public:
	cloud_viewer(const std::string & window_name);

	cloud_viewer(const std::string & window_name, std::map<std::string, std::string>& config_parameters);

	~cloud_viewer();

	// create a node by point_3d with color
	osg::ref_ptr<osg::Geode> add_point_cloud_with_color(std::vector<point_3d> & points, Eigen::Matrix4f transform = Eigen::Matrix4f::Identity(),
		float r = 0, float g = 0, float b = 0);

	// create a node by point_3d without color
	osg::ref_ptr<osg::Geode> add_point_cloud(std::vector<point_3d> & points, Eigen::Matrix4f transform = Eigen::Matrix4f::Identity());

	void create_display_window(const std::string & window_name);

	//void add_lines(std::vector<point_3d> & points, float line_width = 3.0, float r = 0, float g = 0, float b = 0);

	void update_line(std::vector<point_3d> & line_segment, float r = 0, float g = 0, float b = 0, float line_width = 4.0);

	void update_plane(std::vector<point_3d> & plane_square, float r = 0, float g = 0, float b = 0);

	void update_cylinder(cylinder_func &cf, Eigen::Vector3f & rotated_axis, float rotated_angle, float r, float g, float b, float w);

	void update_cylinder(std::vector<point_3d>& cylinder_points, float r = 0, float g = 0, float b = 0, float point_size = 4.0);

	void update_cylinder_centriod_point_on_bottom(std::vector<point_3d>& points, float r = 0, float g = 0, float b = 0, float point_size = 4.0);

	// update selected point cloud
	void update_selected_point_cloud(std::vector<point_3d>& points, float r, float g, float b, float point_size);

	// update hover point(s) in real time
	void update_hover_point_cloud(std::vector<point_3d>& points, float r, float g, float b, float point_size);

	// update reading point cloud in fine and coarse registration
	void update_reading_point_cloud(std::vector<point_3d>& points, float r, float g, float b, float point_size);

	// update testing points
	void update_testing_point_cloud(std::vector<point_3d>& points, float r, float g, float b, float point_size);

	void add_model(const std::string & filename);

	void display();

	void set_target_points(std::vector<point_3d> * points);

	void set_the_interface_command(interface_command * ic_ptr);

	//void get_picked_points(std::vector<point_3d> & picked_points);

	std::vector<point_3d> * get_target_points();

	void clear_picked_points();

	void clear_shapes();

	void save_points_to_vec(std::vector<point_3d> & points, const std::string & marked_name, std::map < std::string, std::vector<point_3d>> & _m);

	void print_marked_info();

	void set_export_file_name(const std::string & efn);

public:
	// interface command pointer, TODO: to be shared pointer
	interface_command * m_ic_ptr;

	cloud_fitting m_cf;

	std::string m_export_file_name;

	// update once picking, this is current points
	std::vector<point_3d> m_points;

	std::vector<point_3d> m_line_points;

	std::vector<point_3d> m_plane_points;

	std::vector<point_3d> m_cylinder_points;

	std::map <std::string, std::vector<point_3d>> m_marked_points_vec;

	std::vector<point_3d> m_picked_points;

	void export_points();

private:
	osg::ref_ptr<PickHandler> m_selector;

	osg::ref_ptr<osg::Group> m_root;

	// for hover
	osg::ref_ptr<osg::Geode> m_geode_hover_point;

	// only one geometry, which used to show selected points, hooked on this geode, 
	osg::ref_ptr<osg::Geode> m_geode_selected_point_cloud;

	// only one geometry, which used to show fitted line points, hooked on this geode.
	osg::ref_ptr<osg::Geode> m_geode_fitted_line;

	// after picking, update on screen in real time
	osg::ref_ptr<osg::Geode> m_geode_fitted_plane;

	// after picking, update on screen in real time
	osg::ref_ptr<osg::Geode> m_geode_fitted_cylinder;

	// in step of marking cylinder, it used to show the center point on bottom of cylinder
	osg::ref_ptr<osg::Geode> m_geode_fitted_cylinder_centriod_point_on_bottom;

	// update in real time in program "fine registration".
	osg::ref_ptr<osg::Geode> m_geode_reading_point_cloud;

	osg::ref_ptr<osgViewer::Viewer> m_viewer;

	std::vector<point_3d> * m_target_points_ptr;

	// below could be deleted after releasing
	// for testing
	osg::ref_ptr<osg::Geode> m_geode_testing;

	void initialize_geode();

	osg::ref_ptr<osg::Node> cretate_bounding_box(osg::Node * node);

	// kd-tree
public:
	kd_tree m_kdtree;

	// parameters
public:
	viewer_parameters m_viewer_parameters;

private:
	// load parameters from file
	void load_parameters(std::map<std::string, std::string> & parameters);
};
#endif // !cloud_viewer_H
