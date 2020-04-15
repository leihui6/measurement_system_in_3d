#ifndef cloud_viewer_H
#define cloud_viewer_H

#include <windows.h>
//#include <WinDef.h> 
//#include <WinGDI.h>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osg/MatrixTransform>
#include <osg/Geometry>
#include <osg/Geode>

#include "cloud_point.h"

class cloud_viewer
{
public:
	cloud_viewer(const std::string & window_name);

	~cloud_viewer();

	// TODO
	void add_point_cloud(std::vector<point_3d> & points, Eigen::Matrix4f transform = Eigen::Matrix4f::Identity());

	void display();
private:
	osg::ref_ptr<osg::Group> m_root;

	osg::ref_ptr<osgViewer::Viewer> m_viewer;
};
#endif // !cloud_viewer_H


