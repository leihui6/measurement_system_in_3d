#ifndef CLOUD_VIEWER_H
#define CLOUD_VIEWER_H

#include <map>

#include "common_use.h"
#include "cloud_fitting.h"
#include "cloud_geometry.h"
#include "qviewerwidget.h"

/*
 * list of point cloud
 * - hover_point_cloud
 * - picked_point_cloud
 *
*/

// please use these name as a point cloud unique id
#define HOVER_POINT "hover_point_cloud"
#define PICKED_POINTS "picked_point_cloud"

#define POINT_CLOUD "point_cloud"
#define FITTING_CLOUD "fitting_point_cloud"

enum DETECT_TYPE
{
    DT_POINT,
    DT_LINE,
    DT_PLANE,
    DT_CYLINDER
};

class PickHandler;

/*
 *
 * cloud_viewer
 *
 */
class cloud_viewer : public osgGA::GUIEventHandler
{
public:
	cloud_viewer(osg::ref_ptr<osgViewer::Viewer> viewer);
	~cloud_viewer();

public:
    // add/remove by a name
	size_t add_point_cloud(std::vector<point_3d> & points, const std::string & point_cloud_name, float w);
    void remove_point_cloud(const std::string & point_cloud_name);
    bool hide_point_cloud(const std::string & point_cloud_name);

	void set_color(const std::string & point_cloud_name, float r, float g, float b, float w);
	void set_color(const std::string & point_cloud_name, osg::Vec4 & color);

	void set_point_size(const std::string & point_cloud_name, float point_size);

	void set_background_color(float r, float g, float b, float w);
	void set_background_color(osg::Vec4 & color);

	void set_target_point_cloud(std::vector<point_3d> & points);
	std::vector<point_3d> * get_target_point_cloud();

    // default running
	void add_pick_handler();
    // in fact, it is a fake switch
	void remove_pick_handler();

    void clear_labeled_fitting();

	void set_pick_handle(bool is_open);
	bool get_pick_handle_status();

    // get labeled points
    std::map<std::string, std::vector<point_3d> > &get_labeled_points_map();

private:
	bool is_pick_handler;
	// initialize all node will be shown on screen
	void initial_visualized_node();
	// it will be called after adding/replacing with a new point cloud
	void update(const std::string & point_cloud_name, osg::ref_ptr<osg::Node> node);
	// initialized by constructor
	osg::ref_ptr<osgViewer::Viewer> m_viewer;
	std::shared_ptr<std::vector<point_3d>> m_target_point_cloud;
	std::map<std::string, osg::ref_ptr<osg::Node>> m_node_map;

	// normal point cloud properties

public:
	void set_point_cloud_size(float size);
	float get_point_cloud_size();
private:
	float m_point_cloud_size;
	osg::Vec4 m_point_cloud_color;
	osg::Vec4 m_background_color;

	// picked points properties
public:
	float get_picking_range() const;
	osg::Vec4 & get_picked_color();
	float get_picked_size() const;
	std::vector<point_3d> & get_picked_points();
	void set_picked_color(const osg::Vec4 & c);
private:
	float m_picked_size;
	float m_picking_range;
	osg::Vec4 m_picked_color;
	std::vector<point_3d> m_picked_points;
	osg::ref_ptr<PickHandler> m_pick_handler;

	// picked points properties
public:
	osg::Vec4 & get_hover_color();
	void set_hover_color(const osg::Vec4 & c);
	float get_hover_size() const;
private:
	osg::Vec4 m_hover_color;
	float m_hover_size;

    // fitting
public:
	void fit_picked_point_to_point();
    void fit_picked_point_to_line();
    void fit_picked_point_to_plane();

    void add_line_segment(const point_3d & beg_p, const point_3d & end_p, const std::string & line_name, float line_width);
    void add_plane_square(std::vector<point_3d> & plane_square, const std::string plane_name);

    void set_current_detection_type(DETECT_TYPE dt);
    void record_labeled_points();
	DETECT_TYPE get_current_detection_type();

    void set_fitting_color(const osg::Vec4 & c);
private:
    cloud_fitting m_cf;
    DETECT_TYPE m_detection_type;
	// will be increased automatically and work in current env
	std::vector<std::vector<point_3d>> m_point_points;
    std::vector<std::vector<point_3d>> m_line_points;
    //osg::Vec4 m_line_color;
    float m_line_width;
    osg::Vec4 m_fitting_color;

    std::map<std::string,std::vector<point_3d>> m_labeled_points_map;
};

/*
 *
 *  PickHandler
 *
*/
class PickHandler : public osgGA::GUIEventHandler
{
public:
	PickHandler(cloud_viewer * cloud_viewer);
    virtual ~PickHandler();

	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

private:
	bool get_picked_point(osg::ref_ptr<osgViewer::Viewer> view, float x, float y, point_3d & picked_point);
	bool add_point_to_picked_vector(const point_3d & p);
	bool remove_point_from_picked_vector(const point_3d & p);

private:
	cloud_viewer * m_cloud_viewer;
};

#endif // CLOUD_VIEWER_H
