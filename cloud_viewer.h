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

#include "cloud_geometry.h"

class cloud_viewer;

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

private:
	
	cloud_viewer * m_cloud_viewer;

	std::vector<point_3d> m_picked_points;
};


class cloud_viewer
{
public:
	cloud_viewer(const std::string & window_name);

	~cloud_viewer();

	void add_point_cloud_with_color(std::vector<point_3d> & points, float point_size = 4,
		Eigen::Matrix4f transform = Eigen::Matrix4f::Identity(),
		float r = 0, float g = 0, float b = 0);

	void add_point_cloud(std::vector<point_3d> & points, float point_size = 4,
		Eigen::Matrix4f transform = Eigen::Matrix4f::Identity());

	void add_lines(std::vector<point_3d> & points, float line_width = 3.0, float r = 0, float g = 0, float b = 0);

	void add_model(const std::string & filename);

	void display();

	void set_the_target_points(std::vector<point_3d> &points);

	void get_picked_points(std::vector<point_3d> & picked_points);

	std::vector<point_3d> * get_target_points();

private:
	osg::ref_ptr<PickHandler> m_selector;// = new PickHandler(this);

	osg::ref_ptr<osg::Group> m_root;

	osg::ref_ptr<osgViewer::Viewer> m_viewer;

	std::vector<point_3d> * m_target_points;

	bool m_is_set_target;

private:
	void points_to_geometry_node(std::vector<point_3d> & points, osg::ref_ptr<osg::Geometry> geometry, float r = 0, float g = 0, float b = 0);
};
#endif // !cloud_viewer_H


