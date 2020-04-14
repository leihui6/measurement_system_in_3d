#ifndef OSG_VIEWER_H
#define OSG_VIEWER_H

#include <windows.h>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osgUtil/Optimizer>
#include <osg/Geometry>
#include <osg/Geode>

#include "cloud_point.h"

class osg_viewer
{
public:
	osg_viewer();

	~osg_viewer();

	// TODO
	void add_point_cloud(std::vector<point_3d> & points);

private:
	osg::ref_ptr<osg::Group> root;
};
#endif // !OSG_VIEWER_H


