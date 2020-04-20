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
#include "cloud_point.h"

class PickHandler : public osgGA::GUIEventHandler
{
public:

	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

	void doUserOperations(osgUtil::LineSegmentIntersector::Intersection& result);

};

class cloud_viewer
{
public:
	cloud_viewer(const std::string & window_name);

	~cloud_viewer();

	void add_point_cloud(std::vector<point_3d> & points, Eigen::Matrix4f transform = Eigen::Matrix4f::Identity());

	void add_model(const std::string & filename);

	// just test, removed in the feature.
	void add_test_points();

	void display();
private:
	osg::ref_ptr<osg::Group> m_root;

	osg::ref_ptr<osgViewer::Viewer> m_viewer;
};
#endif // !cloud_viewer_H


