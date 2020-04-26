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
#include <osgUtil/SceneView>
#include <osgUtil/IntersectionVisitor>
#include <osgViewer/Renderer>
#include <osgDB/ReadFile>
#include <osgUtil/Optimizer> 
#include <osgUtil/DelaunayTriangulator> 
#include <osgGA/TrackballManipulator>

#include "cloud_point.h"

class cloud_viewer
{
public:
	cloud_viewer(const std::string & window_name);

	~cloud_viewer();

	void add_point_cloud_with_color(
		std::vector<point_3d> & points,
		Eigen::Matrix4f transform = Eigen::Matrix4f::Identity(),
		float r = 0, float g = 0, float b = 0);

	void add_point_cloud(
		std::vector<point_3d> & points,
		Eigen::Matrix4f transform = Eigen::Matrix4f::Identity());

	void add_model(const std::string & filename);

	// just test, removed in the feature.
	void add_test_points();

	void display();
private:
	osg::ref_ptr<osg::Group> m_root;

	osg::ref_ptr<osgViewer::Viewer> m_viewer;

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
};
#endif // !cloud_viewer_H


