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

class cloud_viewer
{
public:
	cloud_viewer(const std::string & window_name);

	~cloud_viewer();

	void add_point_cloud_with_color(
		std::vector<point_3d> & points, float point_size = 4,
		Eigen::Matrix4f transform = Eigen::Matrix4f::Identity(),
		float r = 0, float g = 0, float b = 0);

	void add_point_cloud(
		std::vector<point_3d> & points, float point_size = 4,
		Eigen::Matrix4f transform = Eigen::Matrix4f::Identity());

	void add_lines(std::vector<point_3d> & points, float r = 0, float g = 0, float b = 0);

	void add_model(const std::string & filename);

	// just test, removed in the feature.
	void add_test_points();

	void display();

	void get_pick_point(const line_func_3d & _line_func_3d, const point_3d & eye_point, point_3d & pick_point, float dis_threshold_with_ray = 0.5);

	void set_the_target_points(std::vector<point_3d> * points);

private:
	osg::ref_ptr<osg::Group> m_root;

	osg::ref_ptr<osgViewer::Viewer> m_viewer;

	std::vector<point_3d> * m_target_points;

	void points_to_geometry_node(std::vector<point_3d> & points, osg::ref_ptr<osg::Geometry> geometry, float r = 0, float g = 0, float b = 0);
};

class PickHandler : public osgGA::GUIEventHandler
{
public:

	PickHandler(cloud_viewer * _cloud_viewer)
	{
		m_cloud_viewer = _cloud_viewer;
	}

	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

	void doUserOperations(osgUtil::LineSegmentIntersector::Intersection& result);

private:
	cloud_viewer * m_cloud_viewer;

	std::vector<point_3d> points_test;

	void screen_to_world(osg::ref_ptr<osgViewer::View> viewer, osg::Vec3d & screen_point, osg::Vec3d & world);

	void get_eye_point(osg::ref_ptr<osgViewer::View> viewer, osg::Vec3d & eye);

	void get_ray_line_func(osg::Vec3d & world, osg::Vec3d & eye, line_func_3d & _line_func_3d);

};
#endif // !cloud_viewer_H


