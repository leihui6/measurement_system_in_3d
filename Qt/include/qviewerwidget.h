#ifndef  QVIEWER_WIDGET_H
#define  QVIEWER_WIDGET_H

#include <QWidget>
#include <QGridLayout>

#include <osgViewer/Viewer>
#include <osgQt/GraphicsWindowQt>
#include <osgUtil/Optimizer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osgUtil/Optimizer>
#include <osg/LineWidth>

#include "cloud_viewer.h"

class cloud_viewer;
class PickHandler; 
enum  DETECT_TYPE;

class QViewerWidget : public QWidget
{
public:
    QViewerWidget(const QRect &geometry);

    virtual ~QViewerWidget();

public:
    void add_pick_handler();

    void remove_pick_handler();

    size_t add_point_cloud(std::vector<point_3d> & points, const std::string & point_cloud_name, float w);

    bool hide_point_cloud(const std::string & point_cloud_name);

    bool show_axis(float scale);

    bool hide_axis();

    void clear_point_cloud();

    void set_color(const std::string & point_cloud_name, float r,float g,float b,float w);

    void set_point_size(const std::string & point_cloud_name, float point_size);

    void set_background_color(float r,float g,float b, float w);

    void set_target_point_cloud(std::vector<point_3d> &points);

	void fit_picked_point_to_point();
    void fit_picked_point_to_line();
    void fit_picked_point_to_plane();
    void fit_picked_point_cylinder();

    void get_labeled_points_map(std::map<std::string,std::vector<point_3d>> & labeled_points_map);
	
    void record_labeled_points();
    void clear_labeled_fitting();

    //void add_lines(std::vector<point_3d> & points, const std::string line_name, float line_width);

	void get_picked_points(std::vector<point_3d>& picked_points);

	DETECT_TYPE get_current_detection_type();

protected:
    osg::ref_ptr<osg::Group> m_scene;

	osg::ref_ptr<osgViewer::Viewer> m_viewer;

    // operation in the viewer
    std::shared_ptr<cloud_viewer> m_cloud_viewer;

private:
    osgQt::GraphicsWindowQt *createGraphicsWindow(const QRect &geometry);

    void initCamera(const QRect &geometry);

    void paintEvent(QPaintEvent *);
};

#endif // QVIEWER_WIDGET_H
