#include "qviewerwidget.h"

QViewerWidget::QViewerWidget(const QRect &geometry)
	: QWidget()
	, m_scene(new osg::Group)
	, m_viewer(new osgViewer::Viewer)
{
    initCamera(geometry);

    m_viewer->setSceneData(m_scene);
    m_viewer->addEventHandler(new osgViewer::StatsHandler);
    m_viewer->setCameraManipulator(new osgGA::TrackballManipulator);
    m_viewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);

    m_cloud_viewer = std::make_shared<cloud_viewer>(m_viewer);

    osgQt::GraphicsWindowQt *gw = static_cast<osgQt::GraphicsWindowQt *>(m_viewer->getCamera()->getGraphicsContext());

    QGridLayout *layout = new QGridLayout;

    if (layout != Q_NULLPTR)
    {
        layout->addWidget(gw->getGLWidget());
        this->setLayout(layout);
    }
}

QViewerWidget::~QViewerWidget()
{

}

void QViewerWidget::add_pick_handler()
{
    if(m_cloud_viewer)
        m_cloud_viewer->add_pick_handler();
}

void QViewerWidget::remove_pick_handler()
{
    if(m_cloud_viewer)
        m_cloud_viewer->remove_pick_handler();
}

size_t QViewerWidget::add_point_cloud(std::vector<point_3d> & points, const std::string & point_cloud_name, float w)
{
    return m_cloud_viewer->add_point_cloud(points,point_cloud_name,w);
}

bool QViewerWidget::hide_point_cloud(const std::string &point_cloud_name)
{
    return m_cloud_viewer->hide_point_cloud(point_cloud_name);
}

bool QViewerWidget::show_axis(float scale)
{
    point_3d p0(0,0,0), p1(1*scale,0,0),p2(0,1*scale,0),p3(0,0,1*scale);
    std::vector<point_3d> line_segment_points{p0, p1, p0, p2, p0, p3};

    m_cloud_viewer->add_line_segment(p0, p1, "axis_x", 4);
    m_cloud_viewer->set_color("axis_x", 255, 0, 0, 1);

    m_cloud_viewer->add_line_segment(p0, p2, "axis_y", 4);
    m_cloud_viewer->set_color("axis_y", 0, 255, 0, 1);

    m_cloud_viewer->add_line_segment(p0, p3, "axis_z", 4);
    m_cloud_viewer->set_color("axis_z", 0, 0, 255, 1);

    return true;
}

bool QViewerWidget::hide_axis()
{
    m_cloud_viewer->hide_point_cloud("axis_x");
    m_cloud_viewer->hide_point_cloud("axis_y");
    m_cloud_viewer->hide_point_cloud("axis_z");

    return true;
}

void QViewerWidget::clear_point_cloud()
{
    m_cloud_viewer->remove_point_cloud(HOVER_POINT);
    m_cloud_viewer->remove_point_cloud(PICKED_POINTS);
}

void QViewerWidget::set_color(const std::string & point_cloud_name, float r, float g, float b, float w)
{
	if (point_cloud_name == HOVER_POINT)
	{
		m_cloud_viewer->set_hover_color(osg::Vec4(r / 255.0, g / 255.0, b / 255.0, w));
	}
	else if (point_cloud_name == PICKED_POINTS)
	{
		m_cloud_viewer->set_picked_color(osg::Vec4(r / 255.0, g / 255.0, b / 255.0, w));
	}
	else
	{
		m_cloud_viewer->set_color(point_cloud_name, r, g, b, w);
	}
}

void QViewerWidget::set_point_size(const std::string & point_cloud_name, float point_size)
{
    m_cloud_viewer->set_point_size(point_cloud_name,point_size);
}

void QViewerWidget::set_background_color(float r,float g,float b, float w)
{
    m_cloud_viewer->set_background_color(r,g,b,w);
}

void QViewerWidget::set_target_point_cloud(std::vector<point_3d> &points)
{
    m_cloud_viewer->set_target_point_cloud(points);
}

void QViewerWidget::fit_picked_point_to_point()
{
	m_cloud_viewer->set_current_detection_type(DT_POINT);
	m_cloud_viewer->fit_picked_point_to_point();
}

void QViewerWidget::fit_picked_point_to_line()
{
    m_cloud_viewer->set_current_detection_type(DT_LINE);
    m_cloud_viewer->fit_picked_point_to_line();
}

void QViewerWidget::get_labeled_points_map(std::map<std::string,std::vector<point_3d>> & labeled_points_map)
{
    labeled_points_map = m_cloud_viewer->get_labeled_points_map();
}

void QViewerWidget::record_labeled_points()
{
    m_cloud_viewer->record_labeled_points();
}

void QViewerWidget::clear_labeled_fitting()
{
    m_cloud_viewer->clear_labeled_fitting();
}

void QViewerWidget::get_picked_points(std::vector<point_3d>& picked_points)
{
	picked_points = m_cloud_viewer->get_picked_points();
}

DETECT_TYPE QViewerWidget::get_current_detection_type()
{
	return m_cloud_viewer->get_current_detection_type();
}

osgQt::GraphicsWindowQt *QViewerWidget::createGraphicsWindow(const QRect &geometry)
{
    osg::DisplaySettings *ds = osg::DisplaySettings::instance().get();

    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->windowName = "";
    traits->windowDecoration = false;
    traits->x = geometry.x();
    traits->y = geometry.y();
    traits->width = geometry.width();
    traits->height = geometry.height();

    if (traits->height == 0) traits->height = 1;

    traits->doubleBuffer = true;
    traits->alpha = ds->getMinimumNumAlphaBits();
    traits->stencil = ds->getMinimumNumStencilBits();
    traits->sampleBuffers = ds->getMultiSamples();
    traits->samples = ds->getNumMultiSamples();

    return new osgQt::GraphicsWindowQt(traits.get());
}

void QViewerWidget::initCamera(const QRect &geometry)
{
    osg::Camera *camera = m_viewer->getCamera();

    osg::ref_ptr<osgQt::GraphicsWindowQt> gw = createGraphicsWindow(geometry);
    gw->setTouchEventsEnabled(true);
    camera->setGraphicsContext(gw.get());

    const osg::GraphicsContext::Traits *traits = gw->getTraits();
    //camera->setClearColor(osg::Vec4(0.7f, 0.7f, 0.7f, 1.0f));
    std::cout << traits->width <<" "<< traits->height<<std::endl;
    camera->setViewport(0, 0, traits->width, traits->height);

    double aspect = static_cast<double>(traits->width) / static_cast<double>(traits->height);
    camera->setProjectionMatrixAsPerspective(30.0, aspect, 1.0, 1000.0);
    GLenum buffer = (traits->doubleBuffer) ? GL_BACK : GL_FRONT;
    camera->setDrawBuffer(buffer);
    camera->setReadBuffer(buffer);

    // Solve the problem that one point in geode does not show
    osg::CullStack::CullingMode cullingMode = camera->getCullingMode();
    cullingMode &= ~(osg::CullStack::SMALL_FEATURE_CULLING);
    camera->setCullingMode(cullingMode);
}

void QViewerWidget::paintEvent(QPaintEvent *)
{
    m_viewer->frame();
}
