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

	void add_point_cloud(std::vector<point_3d> & points);

};
#endif // !OSG_VIEWER_H


