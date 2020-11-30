#include "cloud_viewer.h"


cloud_viewer::cloud_viewer(const std::string & window_name, std::map<std::string, std::string>& config_parameters)
	: m_root(new osg::Group),
	m_viewer(new osgViewer::Viewer),
	m_target_points_ptr(nullptr),
	m_ic_ptr(nullptr)
{
	m_selector = new PickHandler;
	m_selector->set_viewer_ptr(this);

	load_parameters(config_parameters);

	// create a display window and initialize the camera 
	create_display_window(window_name);

	// set the handler that controls the selection operation
	//m_selector = new PickHandler(this);

	initialize_geode();
}

cloud_viewer::cloud_viewer(const std::string & window_name)
	: m_root(new osg::Group),
	m_viewer(new osgViewer::Viewer),
	m_target_points_ptr(nullptr),
	m_ic_ptr(nullptr),
	m_selector(new PickHandler)
{
	// create a display window and initialize the camera 
	create_display_window(window_name);

	// set the handler that controls the selection operation
	m_selector->set_viewer_ptr(this);

	initialize_geode();
}

cloud_viewer::~cloud_viewer()
{
	// TODO: manage pointer in case of leaking of memory
}

osg::ref_ptr<osg::Geode> cloud_viewer::add_point_cloud_with_color(std::vector<point_3d> & points, Eigen::Matrix4f t, float r, float g, float b)
{
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();

	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();

	points_to_geometry_node(points, geometry, r, g, b);

	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, points.size()));
	osg::StateSet* stateSet = geometry->getOrCreateStateSet();
	osg::Point* state_point_size = new osg::Point;
	state_point_size->setSize(m_viewer_parameters.point_size);
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

	return geode;
}

osg::ref_ptr<osg::Geode> cloud_viewer::add_point_cloud(std::vector<point_3d> & points, Eigen::Matrix4f t)
{
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();

	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();

	points_to_geometry_node(points, geometry);

	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, points.size()));
	osg::StateSet* stateSet = geometry->getOrCreateStateSet();
	osg::Point* state_point_size = new osg::Point;
	state_point_size->setSize(m_viewer_parameters.point_size);
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

	return geode;
}

void cloud_viewer::update_selected_point_cloud(std::vector<point_3d>& points, float r, float g, float b, float point_size)
{
	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();

	points_to_geometry_node(points, geometry, r, g, b);

	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, points.size()));

	osg::StateSet* stateSet = geometry->getOrCreateStateSet();
	osg::Point* state_point_size = new osg::Point;
	state_point_size->setSize(point_size);
	stateSet->setAttribute(state_point_size);

	//osg::ref_ptr<osg::Node> points_bounding_box = cretate_bounding_box(geometry);

	osg::ref_ptr<osg::Group> group_nodes = new osg::Group();

	group_nodes->addChild(geometry);
	//group_nodes->addChild(points_bounding_box);

	m_geode_selected_point_cloud->setChild(0, group_nodes);
}

void cloud_viewer::update_testing_point_cloud(std::vector<point_3d>& points, float r, float g, float b, float point_size)
{
	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();

	points_to_geometry_node(points, geometry, r, g, b);

	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, points.size()));

	osg::StateSet* stateSet = geometry->getOrCreateStateSet();
	osg::Point* state_point_size = new osg::Point;
	state_point_size->setSize(point_size);
	stateSet->setAttribute(state_point_size);

	m_geode_testing->setChild(0, geometry);
}

void cloud_viewer::update_hover_point_cloud(std::vector<point_3d>& points, float r, float g, float b, float point_size)
{
	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();

	points_to_geometry_node(points, geometry, r, g, b);

	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, points.size()));

	osg::StateSet* stateSet = geometry->getOrCreateStateSet();
	osg::Point* state_point_size = new osg::Point;
	state_point_size->setSize(point_size);
	stateSet->setAttribute(state_point_size);

	m_geode_hover_point->setChild(0, geometry);
}

void cloud_viewer::create_display_window(const std::string & window_name)
{
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
	camera->setClearColor(m_viewer_parameters.background_color);

	// Solve the problem that one point in geode does not show
	osg::CullStack::CullingMode cullingMode = camera->getCullingMode();
	cullingMode &= ~(osg::CullStack::SMALL_FEATURE_CULLING);
	camera->setCullingMode(cullingMode);
}

//void cloud_viewer::add_lines(std::vector<point_3d>& points, float line_width, float r,float g,float b)
//{
//	osg::ref_ptr<osg::Geode> geode = new osg::Geode();
//
//	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
//
//	points_to_geometry_node(points, geometry, r, g, b);
//
//	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, points.size()));
//
//	osg::ref_ptr<osg::LineWidth> lw = new osg::LineWidth(line_width);
//	
//	geometry->getOrCreateStateSet()->setAttribute(lw, osg::StateAttribute::ON);
//
//	geode->addDrawable(geometry);
//
//	m_root->addChild(geode.get());
//}

void cloud_viewer::update_line(std::vector<point_3d> & line_segment, float r, float g, float b, float line_width)
{
	if (line_segment.empty())
	{
		osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
		m_geode_fitted_line->setChild(0, geometry);
		return;
	}

	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
	points_to_geometry_node(line_segment, geometry, r, g, b);
	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, line_segment.size()));
	osg::ref_ptr<osg::LineWidth> lw = new osg::LineWidth(line_width);
	geometry->getOrCreateStateSet()->setAttribute(lw, osg::StateAttribute::ON);
	m_geode_fitted_line->setChild(0, geometry);
}

void cloud_viewer::update_plane(std::vector<point_3d> & plane_square,float r, float g, float b)
{
	if (plane_square.empty())
	{
		osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
		m_geode_fitted_plane->setChild(0, geometry);
		return;
	}

	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
	points_to_geometry_node(plane_square, geometry, r, g, b, m_viewer_parameters.fitting_plane_transparency);
	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON, 0, plane_square.size()));
	geometry->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	geometry->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
	m_geode_fitted_plane->setChild(0, geometry);
}

void cloud_viewer::update_cylinder(cylinder_func &cf, Eigen::Vector3f & rotated_axis, float rotated_angle, float r, float g, float b, float w)
{
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();

	point_3d 
		center_p = cf.axis.get_origin_point_3d(),
		direction = cf.axis.get_direction_point_3d();


	osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(
		new osg::Cylinder
		(
			osg::Vec3(0, 0, 0),
			cf.radius,
			cf.height
		)
	);

	sd->setColor(osg::Vec4(r, g, b, w));
	sd->setColorBinding(osg::Geometry::BIND_OVERALL);

	osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
	normals->push_back(osg::Vec3(direction.x, direction.y, direction.z));

	sd->setNormalArray(normals);
	sd->setNormalBinding(osg::Geometry::BIND_OVERALL);

	osg::StateSet* stateset = sd->getOrCreateStateSet();
	stateset->setMode(GL_BLEND, osg::StateAttribute::ON);

	osg::Matrix mRotate(osg::Matrix::identity()), mTrans(osg::Matrix::identity());

	mTrans.makeTranslate(osg::Vec3f(center_p.x, center_p.y, center_p.z));

	mRotate.makeRotate(osg::inDegrees(rotated_angle), rotated_axis[0], rotated_axis[1], rotated_axis[2]);

	osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform(osg::Matrix::identity());

	mt->setMatrix(mRotate*mTrans);

	mt->addChild(sd);

	geode->addChild(mt);

	m_geode_fitted_cylinder->setChild(0, geode);
}

void cloud_viewer::update_cylinder(std::vector<point_3d>& cylinder_points, float r, float g, float b,float point_size)
{
	if (cylinder_points.empty())
	{
		osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();

		m_geode_fitted_cylinder->setChild(0, geometry);

		return;
	}

	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
	points_to_geometry_node(cylinder_points, geometry, r, g, b);
	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, cylinder_points.size()));

	osg::StateSet* stateSet = geometry->getOrCreateStateSet();
	osg::Point* state_point_size = new osg::Point;
	state_point_size->setSize(point_size);
	stateSet->setAttribute(state_point_size);

	m_geode_fitted_cylinder->setChild(0, geometry);
}

void cloud_viewer::update_cylinder_centriod_point_on_bottom(std::vector<point_3d>& points, float r, float g, float b, float point_size)
{
	if (points.empty())
	{
		osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();

		m_geode_fitted_cylinder_centriod_point_on_bottom->setChild(0, geometry);

		return;
	}
	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();

	points_to_geometry_node(points, geometry, r, g, b);

	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, points.size()));

	osg::StateSet* stateSet = geometry->getOrCreateStateSet();
	osg::Point* state_point_size = new osg::Point;
	state_point_size->setSize(point_size);
	stateSet->setAttribute(state_point_size);

	m_geode_fitted_cylinder_centriod_point_on_bottom->setChild(0, geometry);
}

void cloud_viewer::update_reading_point_cloud(std::vector<point_3d>& points, float r, float g, float b, float point_size)
{
	if (points.empty())
	{
		osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();

		m_geode_reading_point_cloud->setChild(0, geometry);

		return;
	}
	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();

	points_to_geometry_node(points, geometry, r, g, b);

	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, points.size()));

	osg::StateSet* stateSet = geometry->getOrCreateStateSet();
	osg::Point* state_point_size = new osg::Point;
	state_point_size->setSize(point_size);
	stateSet->setAttribute(state_point_size);

	m_geode_reading_point_cloud->setChild(0, geometry);
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
	std::cout << m_root->getNumChildren() << " node hooked on root." << std::endl;;

	m_viewer->addEventHandler(m_selector.get());

	m_viewer->setSceneData(m_root.get());

	osgUtil::Optimizer optimizer;
	optimizer.optimize(m_root.get());

	//m_viewer->realize();

	m_viewer->run();
}

void cloud_viewer::set_target_points(std::vector<point_3d> * points)
{
	m_target_points_ptr = points;

	m_kdtree.load_points(*points);
}

void cloud_viewer::set_the_interface_command(interface_command * ic_ptr)
{
	m_ic_ptr = ic_ptr;
}

//void cloud_viewer::get_picked_points(std::vector<point_3d>& picked_points)
//{
//	m_selector->get_picked_points(picked_points);
//}

std::vector<point_3d> * cloud_viewer::get_target_points()
{
	return m_target_points_ptr;
}

void cloud_viewer::clear_picked_points()
{
	m_picked_points.clear();

	update_selected_point_cloud(m_picked_points, 255, 255, 255, 10);
}

void cloud_viewer::clear_shapes()
{
	std::vector<point_3d> empty_points;

	update_line(empty_points);

	update_plane(empty_points);

	update_cylinder(empty_points);

	update_cylinder_centriod_point_on_bottom(empty_points);
}

void cloud_viewer::save_points_to_vec(std::vector<point_3d> & points, const std::string & marked_name, std::map < std::string, std::vector<point_3d>> & _m)
{
	if (points.empty())
	{
		return;
	}

	_m[marked_name] = points;
}

void cloud_viewer::print_marked_info()
{
	size_t point_count = 0, line_count = 0, plane_count = 0, cylinder_count = 0;

	std::map <std::string, std::vector<point_3d>>::iterator it;

	for (it = m_marked_points_vec.begin(); it != m_marked_points_vec.end(); it++)
	{
		if (it->first.find("point") != std::string::npos)
		{
			point_count++;
		}
		else if (it->first.find("line") != std::string::npos)
		{
			line_count++;
		}
		else if (it->first.find("plane") != std::string::npos)
		{
			plane_count++;
		}
		else if (it->first.find("cylinder") != std::string::npos)
		{
			cylinder_count++;
		}
	}

	std::cout
		<< "here are information on marked shapes:\n"
		<< "point shapes:" << point_count << "\n"
		<< "line shapes:" << line_count << "\n"
		<< "plane shapes:" << plane_count << "\n"
		<< "cylinder shapes:" << cylinder_count << "\n";
}

void cloud_viewer::set_export_file_name(const std::string & efn)
{
	this->m_export_file_name = efn;
}

void cloud_viewer::export_points()
{
	export_marked_points(m_marked_points_vec, m_export_file_name + "/marked_points.txt");
}

void cloud_viewer::initialize_geode()
{
	// add a empty selected set, it will be replaced after selecting operation
	std::vector<point_3d> empty_point_cloud;
	m_geode_selected_point_cloud = add_point_cloud(empty_point_cloud);

	// add a empty fitted line points
	m_geode_fitted_line = add_point_cloud(empty_point_cloud);

	// add a empty fitted plane points
	m_geode_fitted_plane = add_point_cloud(empty_point_cloud);

	// add a empty fitted cylinder points
	m_geode_fitted_cylinder = add_point_cloud(empty_point_cloud);

	// add a empty testing points
	m_geode_testing = add_point_cloud(empty_point_cloud);

	// add a empty hover points
	m_geode_hover_point = add_point_cloud(empty_point_cloud);

	m_geode_fitted_cylinder_centriod_point_on_bottom = add_point_cloud(empty_point_cloud);

	m_geode_reading_point_cloud = add_point_cloud(empty_point_cloud);
}

osg::ref_ptr<osg::Node> cloud_viewer::cretate_bounding_box(osg::Node * node)
{
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();
	osg::ComputeBoundsVisitor boundVisitor;
	node->accept(boundVisitor);
	osg::BoundingBox boundingBox = boundVisitor.getBoundingBox();

	osg::notify(osg::ALWAYS) << "bouding box info" << std::endl;
	osg::notify(osg::ALWAYS) << "xMax: " << boundingBox.xMax() << std::endl;
	osg::notify(osg::ALWAYS) << "xMin: " << boundingBox.xMin() << std::endl;
	osg::notify(osg::ALWAYS) << "yMax: " << boundingBox.yMax() << std::endl;
	osg::notify(osg::ALWAYS) << "yMin: " << boundingBox.yMin() << std::endl;
	osg::notify(osg::ALWAYS) << "zMax: " << boundingBox.zMax() << std::endl;
	osg::notify(osg::ALWAYS) << "zMin: " << boundingBox.zMin() << std::endl;
	osg::notify(osg::ALWAYS) << "center: x=" << boundingBox.center().x()
		<< ",y=" << boundingBox.center().y()
		<< ",z=" << boundingBox.center().z() << std::endl;

	float length = boundingBox.xMax() - boundingBox.xMin();
	float width = boundingBox.yMax() - boundingBox.yMin();
	float height = boundingBox.zMax() - boundingBox.zMin();
	osg::ref_ptr<osg::ShapeDrawable> drawable = new osg::ShapeDrawable(
		new osg::Box(boundingBox.center(), length, width, height));
	drawable->setColor(osg::Vec4(1.0, 1.0, 0.0, 1.0));

	osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet;
	stateset = drawable->getOrCreateStateSet();
	osg::ref_ptr<osg::PolygonMode> polygonMode = new osg::PolygonMode(
		osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
	stateset->setAttributeAndModes(polygonMode);

	osg::ref_ptr<osg::LineWidth> linewidth = new osg::LineWidth(3.0);
	stateset->setAttribute(linewidth);

	geode->addDrawable(drawable);
	return geode;
}

void cloud_viewer::load_parameters(std::map<std::string, std::string>& parameters)
{
	if (parameters.empty()) return;
	m_viewer_parameters.picking_range = std::stof(parameters["marking_picking_range"]);
	m_viewer_parameters.set_background_color(str_to_vec4(parameters["marking_background_color"]));
	m_viewer_parameters.point_size = std::stof(parameters["marking_point_size"]);
	m_viewer_parameters.point_color = str_to_vec4(parameters["marking_point_color"]);
}
