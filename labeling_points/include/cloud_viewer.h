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

#include "interface_command.h"
#include "cloud_fitting.h"
#include "cloud_geometry.h"

class cloud_viewer;

struct osg_point_structure
{
	osg_point_structure() :
		coords(new osg::Vec3Array()),
		colors(new osg::Vec4Array()),
		normals(new osg::Vec3Array()) {}

	osg::ref_ptr<osg::Vec3Array> coords;
	osg::ref_ptr<osg::Vec4Array> colors;
	osg::ref_ptr<osg::Vec3Array> normals;
};

//! pick point on point cloud
class PickHandler : public osgGA::GUIEventHandler
{
public:

	PickHandler(cloud_viewer * _cloud_viewer);

	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

	void get_picked_points(std::vector<point_3d> & picked_points);

private:
	
	bool get_picked_point(osg::ref_ptr< osgViewer::View> viewer, float window_x, float window_y, point_3d & p);

	void screen_to_world(osg::ref_ptr<osgViewer::View> viewer, osg::Vec3d & screen_point, osg::Vec3d & world);

	void get_eye_point(osg::ref_ptr<osgViewer::View> viewer, osg::Vec3d & eye);

	void get_ray_line_func(osg::Vec3d & world, osg::Vec3d & eye, line_func_3d & _line_func_3d);

	bool calc_intersection_between_ray_and_points(const line_func_3d & _line_func_3d, const point_3d & eye_point, point_3d & pick_point, float dis_threshold_with_ray);

	void update_shapes();

	void process_line();

	void process_plane();

	void process_cylinder();

private:
	
	cloud_viewer * m_cloud_viewer;

	std::vector<point_3d> m_picked_points;

	bool add_point_to_picked_vector(const point_3d & p);

	bool remove_point_from_picked_vector(const point_3d & p);
};

class cloud_viewer
{
public:
	cloud_viewer(const std::string & window_name);

	~cloud_viewer();

	// create a node by point_3d with color
	osg::ref_ptr<osg::Geode> add_point_cloud_with_color(std::vector<point_3d> & points, float point_size = 4,
		Eigen::Matrix4f transform = Eigen::Matrix4f::Identity(),
		float r = 0, float g = 0, float b = 0);

	// create a node by point_3d without color
	osg::ref_ptr<osg::Geode> add_point_cloud(std::vector<point_3d> & points, float point_size = 4,
		Eigen::Matrix4f transform = Eigen::Matrix4f::Identity());

	// update selected point cloud
	void update_selected_point_cloud(std::vector<point_3d>& points, float r, float g, float b, float point_size);

	// update testing points
	void update_testing_point_cloud(std::vector<point_3d>& points, float r, float g, float b, float point_size);

	void create_display_window(const std::string & window_name);

	//void add_lines(std::vector<point_3d> & points, float line_width = 3.0, float r = 0, float g = 0, float b = 0);
	
	void update_line(std::vector<point_3d> & line_segment, float r = 0, float g = 0, float b = 0, float line_width = 4.0);

	void update_plane(std::vector<point_3d> & plane_hull, float r = 0, float g = 0, float b = 0);

	void add_model(const std::string & filename);

	void display();

	void set_the_target_points(std::vector<point_3d> & points);

	void set_the_interface_command(interface_command * ic_ptr);

	void get_picked_points(std::vector<point_3d> & picked_points);

	std::shared_ptr<std::vector<point_3d>> get_target_points();

public:

	// interface command pointer, TODO: to be shared pointer
	interface_command * m_ic_ptr;

	cloud_fitting m_cf;

private:
	osg::ref_ptr<PickHandler> m_selector;

	osg::ref_ptr<osg::Group> m_root;

	// only one geometry, which used to show selected points, hooked on this geode, 
	osg::ref_ptr<osg::Geode> m_geode_selected_point_cloud;

	// only one geometry, which used to show fitted line points, hooked on this geode.
	osg::ref_ptr<osg::Geode> m_geode_fitted_line;

	// only one geometry, which used to show fitted line points, hooked on this geode.
	osg::ref_ptr<osg::Geode> m_geode_fitted_plane;

	// for testing
	osg::ref_ptr<osg::Geode> m_geode_testing;

	osg::ref_ptr<osgViewer::Viewer> m_viewer;

	std::shared_ptr<std::vector<point_3d>> m_target_points_ptr;
};
#endif // !cloud_viewer_H


