#ifndef CLOUD_PICKHANDLER
#define CLOUD_PICKHANDLER

#include "cloud_viewer.h"

class cloud_viewer;

//! pick point on point cloud
class PickHandler : public osgGA::GUIEventHandler
{
public:
	PickHandler();
	
	//PickHandler(cloud_viewer * _cloud_viewer);
	void set_viewer_ptr(cloud_viewer * _cloud_viewer);

	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

	//void get_picked_points(std::vector<point_3d> & picked_points);

private:

	bool get_picked_point(osg::ref_ptr< osgViewer::View> viewer, float window_x, float window_y, point_3d & p);

	void screen_to_world(osg::ref_ptr<osgViewer::View> viewer, osg::Vec3d & screen_point, osg::Vec3d & world);

	void get_eye_point(osg::ref_ptr<osgViewer::View> viewer, osg::Vec3d & eye);

	void get_ray_line_func(osg::Vec3d & world, osg::Vec3d & eye, line_func_3d & _line_func_3d);

	bool calc_intersection_between_ray_and_points(const line_func_3d & _line_func_3d, const point_3d & eye_point, point_3d & pick_point, float dis_threshold_with_ray);

	void update_shapes();

	void process_line();

	void process_plane(plane_func_3d & plane_func);

	void process_cylinder();

	void process_point();

public:

	point_3d m_centriod_point_on_bottom;

	cylinder_func m_cylinder_func;

private:

	cloud_viewer * m_cloud_viewer;

	bool add_point_to_picked_vector(const point_3d & p);

	void add_points_to_picked_vector(std::vector< point_3d > & vec);

	bool remove_point_from_picked_vector(const point_3d & p);
};

#endif CLOUD_PICKHANDLER