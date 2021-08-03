#include "cloud_viewer.h"

cloud_viewer::cloud_viewer(osg::ref_ptr<osgViewer::Viewer> viewer)
	:m_viewer(viewer)
	, is_pick_handler(false)
	, m_target_point_cloud(nullptr)
	, m_pick_handler(new PickHandler(this))
{
	m_viewer->addEventHandler(m_pick_handler);

	// normal point cloud
	m_point_cloud_size = 1.0f;
	//m_point_cloud_color = osg::Vec4(0.0,0.0,0.0,1.0);
	//m_background_color = osg::Vec4(135,206,235,1);
	//set_background_color(m_background_color);

	// hover properties
	m_hover_size = m_point_cloud_size * 3;
	m_hover_color = osg::Vec4(255.0, 255.0, 255.0, 1.0);

	// picking properties
	m_picking_range = 0.1f;
	m_picked_size = m_point_cloud_size * 1.5f;
	m_picked_color = osg::Vec4(255.0, 0.0, 0.0, 1.0);

	// fitting line properties
    //m_line_color = osg::Vec4(0.0, 255.0, 0.0, 1.0);
	m_line_width = 4.0f;

    m_fitting_color = osg::Vec4(0.0, 255.0, 0.0, 1.0);

	initial_visualized_node();
}

cloud_viewer::~cloud_viewer()
{

}

size_t cloud_viewer::add_point_cloud(std::vector<point_3d> & points, const std::string & point_cloud_name, float w)
{
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();

    points_to_geometry_node(points, geometry, w);
    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, int(points.size())));
    geode->addDrawable(geometry);

    update(point_cloud_name, geode);
    set_color(point_cloud_name, m_point_cloud_color);
    // set_point_size(point_cloud_name, m_point_cloud_size);

    return m_node_map.size();
}

void cloud_viewer::set_point_size(const std::string & point_cloud_name, float point_size)
{
	if (point_cloud_name == POINT_CLOUD)
	{
		set_point_cloud_size(point_size);
	}

    auto it = m_node_map.find(point_cloud_name);
    if (it == m_node_map.end()) return;

    osg::StateSet* stateSet = it->second->asGeode()->getChild(0)->asGeometry()->getOrCreateStateSet();
    stateSet->setAttribute(new osg::Point(point_size));
}

void cloud_viewer::set_color(const std::string & point_cloud_name, float r,float g,float b,float w)
{
	if (m_node_map.find(point_cloud_name) == m_node_map.end()) return;

    osg::ref_ptr<osg::Node> node = m_node_map[point_cloud_name];
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
    colors->push_back(osg::Vec4(r/255.0f,g/255.0f,b/255.0f,w));

    // color
    node->asGeode()->getChild(0)->asGeometry()->setColorArray(colors.get());
    node->asGeode()->getChild(0)->asGeometry()->setColorBinding(osg::Geometry::BIND_OVERALL);
}

void cloud_viewer::set_color(const std::string & point_cloud_name, osg::Vec4 & color)
{
	if (m_node_map.find(point_cloud_name) == m_node_map.end()) return;

    osg::ref_ptr<osg::Node> node = m_node_map[point_cloud_name];
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
    //color.set( color.r()/255.0f,color.g()/255.0f,color.b()/255.0f,color.w());

    colors->push_back(color);

    node->asGeode()->getChild(0)->asGeometry()->setColorArray(colors.get());
    node->asGeode()->getChild(0)->asGeometry()->setColorBinding(osg::Geometry::BIND_OVERALL);
}

void cloud_viewer::set_background_color(float r, float g, float b, float w)
{
    osg::ref_ptr<osg::Camera> camera = m_viewer->getCamera();
    camera->setClearColor(osg::Vec4(r/255.0f,g/255.0f,b/255.0f,w));
}

void cloud_viewer::set_background_color(osg::Vec4 &color)
{
    osg::ref_ptr<osg::Camera> camera = m_viewer->getCamera();
    color.set( color.r()/255.0f,color.g()/255.0f,color.b()/255.0f,color.w());
    camera->setClearColor(color);
}

void cloud_viewer::set_target_point_cloud(std::vector<point_3d> &points)
{
    m_target_point_cloud = std::make_shared<std::vector<point_3d>>(points);
}

std::vector<point_3d> * cloud_viewer::get_target_point_cloud()
{
    return m_target_point_cloud.get();
}

void cloud_viewer::add_pick_handler()
{
    set_pick_handle(true);
}

void cloud_viewer::remove_pick_handler()
{
    set_pick_handle(false);
}

void cloud_viewer::clear_labeled_fitting()
{
	remove_point_cloud(FITTING_CLOUD);
}

void cloud_viewer::remove_point_cloud(const std::string &point_cloud_name)
{
    osg::ref_ptr<osg::Node> scene = m_viewer->getSceneData();
    if(!scene) return;

    osg::ref_ptr<osg::Node> empty = new osg::Node;
    // remove when it exists
    if(m_node_map.find(point_cloud_name) != m_node_map.end())
    {
        scene->asGroup()->replaceChild(m_node_map[point_cloud_name].get(),empty);
        m_node_map[point_cloud_name] = empty;
    }

    if (point_cloud_name == PICKED_POINTS)
    {
        m_picked_points.clear();
    }
}

bool cloud_viewer::hide_point_cloud(const std::string &point_cloud_name)
{
    osg::ref_ptr<osg::Node> scene = m_viewer->getSceneData();
    if(!scene) return false;

    if(m_node_map.find(point_cloud_name) != m_node_map.end())
    {
        m_node_map[point_cloud_name]->setNodeMask(0);
    }
    return true;
}

void cloud_viewer::set_pick_handle(bool is_open)
{
    is_pick_handler = is_open;
}

bool cloud_viewer::get_pick_handle_status()
{
    return is_pick_handler;
}

std::map<std::string, std::vector<point_3d> > &cloud_viewer::get_labeled_points_map()
{
    return m_labeled_points_map;
}

void cloud_viewer::fit_picked_point_to_point()
{
    if (m_picked_points.empty()) return;

    std::vector<point_3d> fitting_point;
    fitting_point = m_picked_points;
    add_point_cloud(fitting_point, FITTING_CLOUD, 1);
    set_color(FITTING_CLOUD, m_fitting_color);
}

void cloud_viewer::fit_picked_point_to_line()
{
    if(m_picked_points.size() < 2) return;

    line_func_3d lf;
    m_cf.fitting_line_3d_linear_least_squares(m_picked_points,lf);

    point_3d sphere_center;
    float sphere_r;
    centroid_from_points(m_picked_points, sphere_center);
    //mean_distance_from_point_to_points(m_cloud_viewer->m_picked_points, sphere_center, sphere_r);
    longgest_distance_from_point_to_points(m_picked_points, sphere_center, sphere_r);
	sphere_r = sphere_r + 0.2f*sphere_r;

    // calculate the segment points for visualization
    point_3d beg_p,end_p;
    intersection_line_to_sphere(lf, sphere_center, sphere_r, beg_p, end_p);

    //std::string line_name = "line"+std::to_string(m_line_points.size());
	add_line_segment(beg_p, end_p, FITTING_CLOUD, m_line_width);
    set_color(FITTING_CLOUD, m_fitting_color);
}

void cloud_viewer::fit_picked_point_to_plane()
{
    if(m_picked_points.size() < 3) return;

    plane_func_3d pf;
    m_cf.fitting_plane_3d_linear_least_squares(m_picked_points, pf);

    point_3d min_p, max_p;

    max_min_point_3d_vec(m_picked_points, min_p, max_p);

    pedalpoint_point_to_plane(min_p, pf, min_p);

    pedalpoint_point_to_plane(max_p, pf, max_p);

    // using four points to draw a biggest rectangle
    Eigen::Vector3f diagonal, line_direction;
    diagonal = Eigen::Vector3f(max_p.x - min_p.x, max_p.y - min_p.y, max_p.z - min_p.z);
    line_direction = diagonal.cross(Eigen::Vector3f(pf.a, pf.b, pf.c));

    point_3d mid_point, corner_p1, corner_p2;
    mid_point = point_3d((min_p.x + max_p.x) / 2, (min_p.y + max_p.y) / 2, (min_p.z + max_p.z) / 2);
    float half_dis = diagonal.norm() / 2;

    point_along_with_vector_within_dis(mid_point, line_direction, corner_p1, corner_p2, half_dis);

    std::vector<point_3d> biggest_rectangle{ min_p,max_p,corner_p1,corner_p2 }, drawable_points_ordered;
    make_points_ordered_by_distance(biggest_rectangle, drawable_points_ordered);

    add_plane_square(drawable_points_ordered, FITTING_CLOUD);
    set_color(FITTING_CLOUD, m_fitting_color);
}

void cloud_viewer::fit_picked_point_cylinder()
{
    if(m_picked_points.size() < 6) return;

    cylinder_func cf;
    m_cf.fitting_cylinder_linear_least_squares(m_picked_points,cf);

    Eigen::Vector3f z_axis(0, 0, 1);

    Eigen::Vector3f  rotated_axis = z_axis.cross(cf.axis.direction);

    float rotated_angle = 0.0;
    angle_between_two_vector_3d(z_axis, cf.axis.direction, rotated_angle);

    add_cylinder(cf,rotated_axis,rotated_angle, FITTING_CLOUD);
}

void cloud_viewer::add_line_segment(const point_3d &beg_p, const point_3d &end_p, const std::string & line_name, float line_width)
{
    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
    std::vector<point_3d> line_segment{beg_p,end_p};
    points_to_geometry_node(line_segment, geometry, 1.0f);
    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, int(line_segment.size())));

    osg::ref_ptr<osg::LineWidth> lw = new osg::LineWidth(line_width);
    geometry->getOrCreateStateSet()->setAttribute(lw, osg::StateAttribute::ON);

    osg::ref_ptr<osg::Geode> node = new osg::Geode;
    node->addChild(geometry);

    update(line_name,node);
}

void cloud_viewer::add_plane_square(std::vector<point_3d> &plane_square, const std::string plane_name)
{
    if (plane_square.empty())
    {
        return;
    }

    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
    points_to_geometry_node(plane_square, geometry, 1.0f);
    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, int(plane_square.size())));
    geometry->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
    geometry->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);

    osg::ref_ptr<osg::Geode> node = new osg::Geode;
    node->addChild(geometry);

    update(plane_name,node);
}

void cloud_viewer::add_cylinder(cylinder_func &cf, Eigen::Vector3f & rotated_axis, float rotated_angle, const std::string cylinder_name)
{
    // building a cylind with a standard position
    point_3d
            center_p = cf.axis.get_origin_point_3d(),
            direction = cf.axis.get_direction_point_3d();

        osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(
                new osg::Cylinder
                (
                    osg::Vec3(0, 0, 0),
                    cf.radius,
                    cf.height
                ));
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(direction.x, direction.y, direction.z));
    sd->setColor(m_fitting_color);
    sd->setColorBinding(osg::Geometry::BIND_OVERALL);
    sd->setNormalArray(normals);
    sd->setNormalBinding(osg::Geometry::BIND_OVERALL);

    // building a matrix for cylinder
    osg::Matrix mRotate(osg::Matrix::identity()), mTrans(osg::Matrix::identity());
    mTrans.makeTranslate(osg::Vec3f(center_p.x, center_p.y, center_p.z));
    mRotate.makeRotate(
                double(osg::inDegrees(rotated_angle)),
                double(rotated_axis[0]),
                double(rotated_axis[1]),
                double(rotated_axis[2]));
    osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform(osg::Matrix::identity());
    mt->setMatrix(mRotate*mTrans);
    mt->addChild(sd);

    // add
    osg::ref_ptr<osg::Geode> node = new osg::Geode;
    node->addChild(mt);

    update(cylinder_name,node);
}

void cloud_viewer::set_current_detection_type(DETECT_TYPE dt)
{
    m_detection_type = dt;
}

void cloud_viewer::record_labeled_points()
{
    if(m_picked_points.empty()) return;

	if (m_detection_type == DT_POINT)
	{
		std::string labeled_name = "point" + std::to_string(m_point_points.size());
        m_labeled_points_map[labeled_name] = m_picked_points;

        m_point_points.push_back(m_picked_points);
	}
	else if(m_detection_type == DT_LINE)
    {
        if(m_picked_points.size() < 2)
        {
            show_warning("Detection Line","Picked points are not enough, please pick more points");
            return;
        }
        std::string labeled_name = "line"+std::to_string(m_line_points.size());
        m_labeled_points_map[labeled_name] = m_picked_points;

        m_line_points.push_back(m_picked_points);
    }
	else if (m_detection_type == DT_PLANE)
	{
        if(m_picked_points.size() < 3)
        {
            show_warning("Detection Plane","Picked points are not enough, please pick more points");
            return;
        }
        std::string labeled_name = "plane"+std::to_string(m_plane_points.size());
        m_labeled_points_map[labeled_name] = m_picked_points;

        m_plane_points.push_back(m_picked_points);
	}
    else if (m_detection_type == DT_CYLINDER)
    {
        if(m_picked_points.size() < 6)
        {
            show_warning("Detection Cylinder","Picked points are not enough, please pick more points");
            return;
        }
        std::string labeled_name = "cylinder"+std::to_string(m_cylinder_points.size());
        m_labeled_points_map[labeled_name] = m_picked_points;

        m_cylinder_points.push_back(m_picked_points);
    }
	else
	{

	}
	show_warning("selection information", std::to_string(m_picked_points.size()) + " points was selected.");
}

DETECT_TYPE cloud_viewer::get_current_detection_type()
{
    return m_detection_type;
}

void cloud_viewer::set_fitting_color(const osg::Vec4 &c)
{
    m_fitting_color = c;
}

void cloud_viewer::initial_visualized_node()
{
    std::vector<point_3d> empty_points;

    add_point_cloud(empty_points,HOVER_POINT,1);

    add_point_cloud(empty_points,PICKED_POINTS,1);
}

void cloud_viewer::update(const std::string & point_cloud_name,osg::ref_ptr<osg::Node> node)
{
    osg::ref_ptr<osg::Node> scene = m_viewer->getSceneData();

    if(!scene) return;

    // add new one
    if(m_node_map.find(point_cloud_name) == m_node_map.end())
    {
        scene->asGroup()->addChild(node);
        m_node_map[point_cloud_name] = node;
    }
    // replace the current one
    else
    {
        scene->asGroup()->replaceChild(m_node_map[point_cloud_name].get(),node);
        m_node_map[point_cloud_name] = node;
    }
    //std::cout << m_node_map.size()<<std::endl;
}

void cloud_viewer::set_point_cloud_size(float size)
{
	m_point_cloud_size = size;
}

float cloud_viewer::get_point_cloud_size()
{
	return m_point_cloud_size;
}

float cloud_viewer::get_picking_range() const
{
    return m_picking_range;
}

osg::Vec4 & cloud_viewer::get_picked_color()
{
    return m_picked_color;
}

float cloud_viewer::get_picked_size() const
{
    return m_picked_size;
}

std::vector<point_3d> &cloud_viewer::get_picked_points()
{
    return m_picked_points;
}

void cloud_viewer::set_picked_color(const osg::Vec4 & c)
{
	m_picked_color = c;
}

osg::Vec4 &cloud_viewer::get_hover_color()
{
    return m_hover_color;
}

void cloud_viewer::set_hover_color(const osg::Vec4 & c)
{
	m_hover_color = c;
}

float cloud_viewer::get_hover_size() const
{
    return m_hover_size;
}

PickHandler::PickHandler(cloud_viewer * cloud_viewer)
    :m_cloud_viewer(cloud_viewer)
{
}

PickHandler::~PickHandler()
{
    std::cout << "delete pickhandler"<<std::endl;
}

bool PickHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    if (!m_cloud_viewer->get_pick_handle_status()) return false;

    int pick_status = 0;
    osg::ref_ptr<osgViewer::Viewer> viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
    if(!viewer) return false;

    switch (ea.getEventType())
    {
    // click point
    case osgGA::GUIEventAdapter::DOUBLECLICK:
        pick_status = 1;
        break;
    case osgGA::GUIEventAdapter::PUSH:
        if (ea.getButton() & osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
            pick_status = 2;
        break;
    default:
        break;
    }

    point_3d picked_point;
    if (!get_picked_point(viewer,ea.getX(),ea.getY(),picked_point)) return false;

    //std::cout << picked_point <<std::endl;
    if(pick_status == 1)
    {
        add_point_to_picked_vector(picked_point);
    }
    else if(pick_status == 2)
    {
        remove_point_from_picked_vector(picked_point);
    }
    //std::cout << m_cloud_viewer.get_picked_points().size()<<std::endl;

	// picked points
	m_cloud_viewer->add_point_cloud(m_cloud_viewer->get_picked_points(), PICKED_POINTS, 1);
	m_cloud_viewer->set_point_size(PICKED_POINTS, 7 * m_cloud_viewer->get_point_cloud_size());
	m_cloud_viewer->set_color(PICKED_POINTS, m_cloud_viewer->get_picked_color());

	// hover point
	std::vector<point_3d> hover_point{ picked_point };
	m_cloud_viewer->add_point_cloud(hover_point, HOVER_POINT, 1);
	m_cloud_viewer->set_point_size(HOVER_POINT, 7 * m_cloud_viewer->get_point_cloud_size());
	m_cloud_viewer->set_color(HOVER_POINT, m_cloud_viewer->get_hover_color());

    return false;
}

bool PickHandler::get_picked_point( osg::ref_ptr<osgViewer::Viewer> viewer, float x, float y, point_3d & picked_point)
{
    if(!m_cloud_viewer->get_target_point_cloud()) return false;

    osg::Vec3d window(double(x), double(y), 0), world, eye;

    osg::ref_ptr<osg::Camera> camera = viewer->getCamera();

    osg::Matrix mVPW = camera->getViewMatrix() * camera->getProjectionMatrix();

    mVPW = mVPW * camera->getViewport()->computeWindowMatrix();

    osg::Matrix invertVPW;

    invertVPW.invert(mVPW);

    world = window * invertVPW;

    osg::Vec3d center, up;

    viewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);

    line_func_3d ray_line;

    ray_line.set_xyz(float(eye.x()), float(eye.y()), float(eye.z()));

    ray_line.set_nml(float(world.x() - eye.x()), float(world.y() - eye.y()), float(world.z() - eye.z()));

    std::vector<point_3d> v_points_on_line;

    points_on_line(*m_cloud_viewer->get_target_point_cloud(),
                   v_points_on_line,ray_line,
                   m_cloud_viewer->get_picking_range());

    float min_dis = FLT_MAX; size_t min_i = 0;
    for(size_t i=0;i<v_points_on_line.size();++i)
    {
        float dis = 0.0;
        distance_point_to_point(v_points_on_line[i],ray_line.get_origin_point_3d(),dis);
        if(dis < min_dis)
        {
            min_dis = dis;
            min_i = i;
        }
    }

    if(!v_points_on_line.empty())
    {
        picked_point = v_points_on_line[min_i];
        return true;
    }
    else
    {
        return false;
    }
}

bool PickHandler::add_point_to_picked_vector(const point_3d & p)
{
    for (std::vector<point_3d>::iterator it = m_cloud_viewer->get_picked_points().begin();
         it != m_cloud_viewer->get_picked_points().end(); ++it)
    {
        if (it->x == p.x &&it->y == p.y &&it->z == p.z)
        {
            std::cout << "point existed" << std::endl;
            return false;
        }
    }

    m_cloud_viewer->get_picked_points().push_back(p);

    return true;
}

bool PickHandler::remove_point_from_picked_vector(const point_3d &p)
{
    for (std::vector<point_3d>::iterator it = m_cloud_viewer->get_picked_points().begin();
         it != m_cloud_viewer->get_picked_points().end(); ++it)
    {
        if (it->x == p.x &&it->y == p.y &&it->z == p.z)
        {
            std::cout << "point unclicked from picked set" << std::endl;
            it = m_cloud_viewer->get_picked_points().erase(it);
            return true;
        }
    }

    std::cout << "point doesn't exist in picked set" << std::endl;
    return false;
}
