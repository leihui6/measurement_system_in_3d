#include "cloud_viewer.h"

PickHandler::PickHandler(cloud_viewer * _cloud_viewer)
{
	m_cloud_viewer = _cloud_viewer;
}

bool PickHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (ea.getEventType() != osgGA::GUIEventAdapter::RELEASE ||
		ea.getButton() != osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON ||
		!(ea.getModKeyMask()&osgGA::GUIEventAdapter::MODKEY_CTRL))
	{
		return false;
	}

	osg::ref_ptr< osgViewer::View> viewer = dynamic_cast<osgViewer::View*>(&aa);

	if (viewer)
	{
		point_3d pp; // picked point
		bool is_clicked = get_picked_point(viewer, ea.getX(), ea.getY(), pp);

		if (is_clicked)
		{
			std::cout << "picked point=" << pp << std::endl;
			// darw this point on cloud.
			m_picked_points.push_back(pp);
			m_cloud_viewer->add_point_cloud_with_color(m_picked_points, 10, Eigen::Matrix4f::Identity(), 255, 255, 255);
		}
		else
		{
			std::cout << "No points is in clicked zone." << std::endl;
		}

	}
	return false;
}

void PickHandler::get_picked_points(std::vector<point_3d>& picked_points)
{
	picked_points = m_picked_points;
}

bool PickHandler::get_picked_point(osg::ref_ptr< osgViewer::View> viewer, float window_x, float window_y, point_3d & p)
{
	osg::Vec3d window(window_x, window_y, 0), world, eye;

	line_func_3d _line_func_3d;

	screen_to_world(viewer, window, world);

	get_eye_point(viewer, eye);

	//std::cout << eye.x() << " " << eye.y() << " " << eye.z() << " " << std::endl;

	get_ray_line_func(world, eye, _line_func_3d);

	return calc_intersection_between_ray_and_points(_line_func_3d, to_point_3d(eye), p, 0.1);
}

cloud_viewer::cloud_viewer(const std::string & window_name)
	: m_root(new osg::Group()),
	m_viewer(new osgViewer::Viewer),
	m_is_set_target(false)
{
	//m_viewer->addEventHandler(new osgGA::StateSetManipulator(m_viewer->getCamera()->getOrCreateStateSet()));
	m_viewer->setCameraManipulator(new osgGA::TrackballManipulator);

	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	traits->x = 40;
	traits->y = 40;
	traits->width = 600;
	traits->height = 480;
	traits->windowName = window_name;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());

	osg::ref_ptr<osg::Camera> camera = m_viewer->getCamera();
	camera->setGraphicsContext(gc.get());
	camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
	GLenum buffer = (traits->doubleBuffer) ? GL_BACK : GL_FRONT;
	camera->setDrawBuffer(buffer);
	camera->setReadBuffer(buffer);

	m_selector = new PickHandler(this);

	//m_viewer->addSlave(camera.get());
}

cloud_viewer::~cloud_viewer()
{

}

void cloud_viewer::add_point_cloud_with_color(std::vector<point_3d> & points, float point_size, Eigen::Matrix4f t, float r, float g, float b)
{
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();

	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();

	points_to_geometry_node(points, geometry, r, g, b);

	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, points.size()));
	osg::StateSet* stateSet = geometry->getOrCreateStateSet();
	osg::Point* state_point_size = new osg::Point;
	state_point_size->setSize(point_size);
	stateSet->setAttribute(state_point_size);

	geode->addDrawable(geometry);

	osg::ref_ptr<osg::MatrixTransform> transformation = new osg::MatrixTransform;

	t.transposeInPlace();

	transformation->setMatrix(osg::Matrixf(
		t(0, 0), t(0, 1), t(0, 2), t(0, 3),
		t(1, 0), t(1, 1), t(1, 2), t(1, 3),
		t(2, 0), t(2, 1), t(2, 2), t(2, 3),
		t(3, 0), t(3, 1), t(3, 2), t(3, 3)));

	m_root->addChild(transformation.get());

	transformation->addChild(geode.get());
}

void cloud_viewer::add_point_cloud(std::vector<point_3d> & points, float point_size, Eigen::Matrix4f t)
{
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();

	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();

	points_to_geometry_node(points, geometry);

	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, points.size()));
	osg::StateSet* stateSet = geometry->getOrCreateStateSet();
	osg::Point* state_point_size = new osg::Point;
	state_point_size->setSize(point_size);
	stateSet->setAttribute(state_point_size);

	geode->addDrawable(geometry);

	osg::ref_ptr<osg::MatrixTransform> transformation = new osg::MatrixTransform;

	t.transposeInPlace();

	transformation->setMatrix(osg::Matrixf(
		t(0, 0), t(0, 1), t(0, 2), t(0, 3),
		t(1, 0), t(1, 1), t(1, 2), t(1, 3),
		t(2, 0), t(2, 1), t(2, 2), t(2, 3),
		t(3, 0), t(3, 1), t(3, 2), t(3, 3)));

	m_root->addChild(transformation.get());

	transformation->addChild(geode.get());
}

void cloud_viewer::add_lines(std::vector<point_3d>& points, float line_width, float r,float g,float b)
{
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();

	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();

	points_to_geometry_node(points, geometry, r, g, b);

	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, points.size()));

	osg::ref_ptr<osg::LineWidth> lw = new osg::LineWidth(line_width);
	
	geometry->getOrCreateStateSet()->setAttribute(lw, osg::StateAttribute::ON);

	geode->addDrawable(geometry);

	m_root->addChild(geode.get());
}

void cloud_viewer::add_model(const std::string & filename)
{
	osg::ref_ptr<osg::Node> node;
	if (filename.find(".osg") != std::string::npos)
	{
		node = osgDB::readNodeFile(filename);
	}
	else if (filename.find(".ply") != std::string::npos)
	{
		//node = osgDB::readply(filename);
	}

	m_root->addChild(node.get());
}

void cloud_viewer::display()
{
	std::cout << "display()" << m_root->getNumChildren() << std::endl;;

	m_viewer->addEventHandler(m_selector.get());

	m_viewer->setSceneData(m_root.get());

	//osgUtil::Optimizer optimizer;
	//optimizer.optimize(m_root.get());
	
	m_viewer->realize();
	
	m_viewer->run();
}

void cloud_viewer::set_the_target_points(std::vector<point_3d> &points)
{
	m_target_points = &points;

	m_is_set_target = true;
}

void cloud_viewer::get_picked_points(std::vector<point_3d>& picked_points)
{
	m_selector->get_picked_points(picked_points);
}

std::vector<point_3d> * cloud_viewer::get_target_points()
{
	return m_target_points;
}

void cloud_viewer::points_to_geometry_node(std::vector<point_3d>& points, osg::ref_ptr<osg::Geometry> geometry, float r, float g, float b)
{
	osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array();

	osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array();

	osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;

	normals->push_back(osg::Vec3(0.0f, 1.0f, 0.0f));

	if (r == 0 && g == 0 && b == 0)
	{
		for (size_t i = 0; i < points.size(); i++)
		{
			coords->push_back(osg::Vec3(points[i].x, points[i].y, points[i].z));

			color->push_back(osg::Vec4(points[i].r, points[i].g, points[i].b, 1.0f));
		}
		geometry->setColorArray(color.get());
		geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	}
	else
	{
		for (size_t i = 0; i < points.size(); i++)
		{
			coords->push_back(osg::Vec3(points[i].x, points[i].y, points[i].z));
		}
		color->push_back(osg::Vec4(r, g, b, 1.0f));
		geometry->setColorArray(color.get());
		geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	}

	geometry->setVertexArray(coords.get());

	geometry->setNormalArray(normals);
	geometry->setNormalBinding(osg::Geometry::BIND_OVERALL);
}

void PickHandler::screen_to_world(osg::ref_ptr<osgViewer::View> viewer, osg::Vec3d & screen_point, osg::Vec3d & world)
{
	osg::ref_ptr<osg::Camera> camera = viewer->getCamera();

	osg::Matrix mVPW = camera->getViewMatrix() * camera->getProjectionMatrix();

	mVPW = mVPW * camera->getViewport()->computeWindowMatrix();

	osg::Matrix invertVPW;

	invertVPW.invert(mVPW);

	world = screen_point * invertVPW;
}

void PickHandler::get_eye_point(osg::ref_ptr<osgViewer::View> viewer, osg::Vec3d & eye)
{
	osg::Vec3d center, up;

	viewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);
}

void PickHandler::get_ray_line_func(osg::Vec3d & world, osg::Vec3d & eye, line_func_3d & _line_func_3d)
{
	_line_func_3d.set_xyz(eye.x(), eye.y(), eye.z());

	_line_func_3d.set_nml(world.x() - eye.x(), world.y() - eye.y(), world.z() - eye.z());
}

bool PickHandler::calc_intersection_between_ray_and_points(const line_func_3d & _line_func_3d, const point_3d & eye_point, point_3d & pick_point, float dis_threshold_with_ray)
{
	std::vector<float> distance_vec;

	std::vector<point_3d> & target_points = *m_cloud_viewer->get_target_points();

	distance_points_to_line(target_points, _line_func_3d, distance_vec);

	float min_dis = FLT_MAX;

	size_t pick_point_index = UINT64_MAX;

	for (size_t i = 0; i < distance_vec.size(); ++i)
	{
		if (distance_vec[i] < dis_threshold_with_ray)
		{
			float dis_to_eye;

			distance_point_to_point((target_points)[i], eye_point, dis_to_eye);

			if (dis_to_eye < min_dis)
			{
				min_dis = dis_to_eye;

				pick_point_index = i;
			}
		}
	}

	if (pick_point_index != UINT64_MAX)
	{
		pick_point.set_xyz((target_points)[pick_point_index].x, (target_points)[pick_point_index].y, (target_points)[pick_point_index].z);

		return true;
	}
	else
	{
		pick_point.set_xyz(0, 0, 0);

		return false;
	}
}
