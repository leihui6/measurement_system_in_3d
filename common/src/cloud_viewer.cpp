#include "cloud_viewer.h"

PickHandler::PickHandler(cloud_viewer * _cloud_viewer)
{
	m_cloud_viewer = _cloud_viewer;
}

bool PickHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	// 0: do nothing
	// 1: click the current point
	// 2: cancel the current point
	int pick_status = 0;

	switch (ea.getEventType())
	{
	case osgGA::GUIEventAdapter::DOUBLECLICK:
		pick_status = 1;
		break;

	case osgGA::GUIEventAdapter::PUSH:
		if (ea.getButton() & osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
		{
			pick_status = 2;
		}
		break;

	default:
		break;
	}


	//std::cout << "user's input type is:" << status << std::endl;

	osg::ref_ptr< osgViewer::View> viewer = dynamic_cast<osgViewer::View*>(&aa);
	
	point_3d pp;

	// do nothing 
	if (pick_status == 0)
	{
		if (viewer)
		{
			bool is_clicked = get_picked_point(viewer, ea.getX(), ea.getY(), pp);

			if (is_clicked)
			{
				std::vector<point_3d> hover_point{ pp };
				
				m_cloud_viewer->update_hover_point_cloud(hover_point, 255, 0, 0, 10);
			}
		}
		return false;
	}

	if (viewer)
	{
		bool is_clicked = get_picked_point(viewer, ea.getX(), ea.getY(), pp);

		//std::cout << "picked point=" << pp << std::endl;

		// picking operation
		if (pick_status == 1)
		{
			if (is_clicked)
			{
				if (add_point_to_picked_vector(pp))
				{
					//std::cout << "added done" << std::endl;
				}
			}
			else
			{
				//std::cout << "No points clicked can be collected, please be closer to the point" << std::endl;
			}
		}
		// removing operation
		else if (pick_status == 2)
		{
			if (is_clicked)
			{
				if (remove_point_from_picked_vector(pp))
				{
					std::cout << "removed done" << std::endl;
				}
			}
			else
			{
				std::cout << "No points clicked can be cancelled, please be closer to the point" << std::endl;
			}
		}

		// only test
		// m_picked_points = *m_cloud_viewer->get_target_points();

		// update shapes by user's option
		update_shapes();
		
		// update: showing on screen in real time
		m_cloud_viewer->update_selected_point_cloud(m_cloud_viewer->m_picked_points, 255, 255, 255, 20);

		//std::cout << "The size of selected points=" << m_cloud_viewer->m_picked_points.size() << std::endl;

		return true;
	} // end for viewer

	return false;
}

//void PickHandler::get_picked_points(std::vector<point_3d>& picked_points)
//{
//	picked_points = m_cloud_viewer->m_picked_points;
//}

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
	: m_root(new osg::Group),
	m_viewer(new osgViewer::Viewer),
	m_target_points_ptr(new std::vector<point_3d>()),
	m_ic_ptr(nullptr)
{
	// create a display window and initialize the camera 
	create_display_window(window_name);

	// set the handler that controls the selection operation
	m_selector = new PickHandler(this);

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

cloud_viewer::~cloud_viewer()
{
	// TODO: manage pointer in case of leaking of memory
}

osg::ref_ptr<osg::Geode> cloud_viewer::add_point_cloud_with_color(std::vector<point_3d> & points, float point_size, Eigen::Matrix4f t, float r, float g, float b)
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

	return geode;
}

osg::ref_ptr<osg::Geode> cloud_viewer::add_point_cloud(std::vector<point_3d> & points, float point_size, Eigen::Matrix4f t)
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

	m_geode_selected_point_cloud->setChild(0, geometry);
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
	
	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, line_segment.size()));
	
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

	points_to_geometry_node(plane_square, geometry, r, g, b);

	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, plane_square.size()));

	//osg::ref_ptr<osg::LineWidth> lw = new osg::LineWidth(line_width);

	//geometry->getOrCreateStateSet()->setAttribute(lw, osg::StateAttribute::ON);

	m_geode_fitted_plane->setChild(0, geometry);
}

void cloud_viewer::update_cylinder(point_3d & center_p, float radius, float height, Eigen::Vector3f & rotated_axis, float rotated_angle, float r, float g, float b)
{
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();

	osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(
		new osg::Cylinder
		(
			osg::Vec3(0, 0, 0),
			radius,
			height
		)
	);

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

void cloud_viewer::set_the_target_points(std::vector<point_3d> * points)
{
	m_target_points_ptr = points;
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
		if (it->first.find("point"))
		{
			point_count++;
		}
		else if (it->first.find("line"))
		{
			line_count++;
		}
		else if (it->first.find("plane"))
		{
			plane_count++;
		}
		else if (it->first.find("cylinder"))
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
	std::map <std::string, std::vector<point_3d>>::iterator it;

	std::ofstream point_file(m_export_file_name + "/marked_points.txt");

	if (point_file.is_open())
	{
		for (it = m_marked_points_vec.begin(); it != m_marked_points_vec.end(); it++)
		{
			std::vector<point_3d> &ps = it->second;

			for (size_t j = 0; j < ps.size(); j++)
			{
				point_file << ps[j].x << " " << ps[j].y << " " << ps[j].z << "\n";
			}
			point_file << "#" << it->first << "\n";
		}
		point_file.close();
	}
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

	std::vector<point_3d> * target_points = m_cloud_viewer->get_target_points();

	distance_points_to_line(*target_points, _line_func_3d, distance_vec);

	float min_dis = FLT_MAX;

	size_t pick_point_index = UINT64_MAX;

	for (size_t i = 0; i < distance_vec.size(); ++i)
	{
		if (distance_vec[i] < dis_threshold_with_ray)
		{
			float dis_to_eye;

			distance_point_to_point((*target_points)[i], eye_point, dis_to_eye);

			if (dis_to_eye < min_dis)
			{
				min_dis = dis_to_eye;

				pick_point_index = i;
			}
		}
	}

	if (pick_point_index != UINT64_MAX)
	{
		pick_point.set_xyz(
			(*target_points)[pick_point_index].x, 
			(*target_points)[pick_point_index].y,
			(*target_points)[pick_point_index].z);

		pick_point.set_nxyz(
			(*target_points)[pick_point_index].nx,
			(*target_points)[pick_point_index].ny,
			(*target_points)[pick_point_index].nz);

		pick_point.set_rgb(
			(*target_points)[pick_point_index].r,
			(*target_points)[pick_point_index].g,
			(*target_points)[pick_point_index].b);

		return true;
	}
	else
	{
		pick_point.set_xyz(0, 0, 0);
		pick_point.set_nxyz(0, 0, 0);
		pick_point.set_rgb(0, 0, 0);

		return false;
	}
}

void PickHandler::update_shapes()
{
	if (this->m_cloud_viewer->m_ic_ptr->m_cs == CS_QUIT)
	{
		std::cout << "shapes detection terminated by user\n";

		return;
	}

	if (this->m_cloud_viewer->m_ic_ptr->m_dt == DT_LINE)
	{
		//std::cout << "updating the line model ... " << std::endl;

		process_line();
		
		m_cloud_viewer->m_line_points = m_cloud_viewer->m_picked_points;

		//std::cout << "line model updated done " << std::endl;
	}
	else if (this->m_cloud_viewer->m_ic_ptr->m_dt == DT_PLANE)
	{
		//std::cout << "updating the plane model ... " << std::endl;

		plane_func_3d plane_func;

		process_plane(plane_func);

		m_cloud_viewer->m_plane_points = m_cloud_viewer->m_picked_points;

		//std::cout << "plane model updated done " << std::endl;
	}
	else if (this->m_cloud_viewer->m_ic_ptr->m_dt == DT_CYLINDER)
	{
		//std::cout << "updating the cylinder model ... " << std::endl;

		process_cylinder();

		//std::cout << "cylinder model updated done " << std::endl;
	}
	else if(this->m_cloud_viewer->m_ic_ptr->m_dt == DT_POINT)
	{
		process_point();
	}
	else
	{
		// nothing
	}

}

void PickHandler::process_line()
{
	/*
	The purpose of this code is to show a line with a known line function within a segment represented by begin and end point.

	line_segment_begin should be the minimal point
	line_segment_end should be the maximal point

	1) let (x0,y0.z0) in "line_func_3d" be the middle point among line_segment_begin and line_segment_end
	2) update (x0,y0.z0)
	3) calculate the "t" representing real begin and end point
	4) draw a line from real begin point to real end point
	*/
	if (m_cloud_viewer->m_picked_points.size() < 2)
	{
		return;
	}
	
	line_func_3d line_func;

	m_cloud_viewer->m_cf.fitting_line_3d_linear_least_squares(m_cloud_viewer->m_picked_points, line_func);
	
	point_3d min_p, max_p;

	max_min_point_3d_vec(m_cloud_viewer->m_picked_points, min_p, max_p);

	std::vector<float>t_b, t_e;

	man_min_t_line_function(line_func, min_p, max_p, t_b, t_e);

	float
		min_value_t_b = 0, max_value_t_b = 0,
		min_value_t_e = 0, max_value_t_e = 0;

	max_min_value_array(t_b, min_value_t_b, max_value_t_b);

	max_min_value_array(t_e, min_value_t_e, max_value_t_e);

	// adjust the (x0,y0,z0) in line function
	bool adjusted = false;
	for (float adjust_t = min_value_t_b; adjust_t < max_value_t_e; adjust_t += 0.01)
	{
		float
			tmp_x = line_func.x + adjust_t * line_func.n,
			tmp_y = line_func.y + adjust_t * line_func.m,
			tmp_z = line_func.z + adjust_t * line_func.l;

		if (
			tmp_x < max_p.x && tmp_x > min_p.x &&
			tmp_y < max_p.y && tmp_y > min_p.y &&
			tmp_z < max_p.z && tmp_z > min_p.z)
		{
			line_func.set_xyz(tmp_x, tmp_y, tmp_z);

			// update t that could let (x0,y0.z0) be middle again
			man_min_t_line_function(line_func, min_p, max_p, t_b, t_e);

			adjusted = true;
		}
	}
	if (adjusted == false)
	{
		std::cerr << "[ERROR] fitting line error" << std::endl;

		return;
	}

	// for drawing a real line on screen
	std::vector<point_3d> real_line_segment(2);

	// get the t that could let real begin point be closer to begin point of segment 
	float real_t_b = 0, real_t_e = 0;
	get_appropriate_t(line_func, t_b, min_p, real_t_b);

	get_appropriate_t(line_func, t_e, max_p, real_t_e);

	real_line_segment[0].set_xyz(line_func.x + line_func.n * real_t_b, line_func.y + line_func.m * real_t_b, line_func.z + line_func.l * real_t_b);

	real_line_segment[1].set_xyz(line_func.x + line_func.n * real_t_e, line_func.y + line_func.m * real_t_e, line_func.z + line_func.l * real_t_e);

	m_cloud_viewer->update_line(real_line_segment, 0, 255, 0, 10);
}

void PickHandler::process_plane(plane_func_3d & plane_func)
{
	if (m_cloud_viewer->m_picked_points.size() < 3)
	{
		return;
	}

	m_cloud_viewer->m_cf.fitting_plane_3d_linear_least_squares(m_cloud_viewer->m_picked_points, plane_func);

	//std::cout
	//	<< plane_func.a << " "
	//	<< plane_func.b << " "
	//	<< plane_func.c << " "
	//	<< plane_func.d << " "
	//	<< std::endl;

	point_3d min_p, max_p;

	max_min_point_3d_vec(m_cloud_viewer->m_picked_points, min_p, max_p);

	pedalpoint_point_to_plane(min_p, plane_func, min_p);

	pedalpoint_point_to_plane(max_p, plane_func, max_p);

	// using four points to draw a biggest rectangle
	Eigen::Vector3f 
		diagonal = 
		Eigen::Vector3f(max_p.x - min_p.x, max_p.y - min_p.y, max_p.z - min_p.z),
		line_direction =
		diagonal.cross(Eigen::Vector3f(plane_func.a, plane_func.b, plane_func.c));

	point_3d
		mid_point = point_3d((min_p.x + max_p.x) / 2, (min_p.y + max_p.y) / 2, (min_p.z + max_p.z) / 2),
		corner_p1, corner_p2;

	float half_dis = diagonal.norm() / 2;

	point_along_with_vector_within_dis(mid_point, line_direction, corner_p1, corner_p2, half_dis);

	//std::vector<point_3d> test_vec{ mid_point ,corner_p1 , corner_p2, min_p, max_p };
	//m_cloud_viewer->update_testing_point_cloud(test_vec, 255, 0, 0, 20);

	std::vector<point_3d> biggest_rectangle{ min_p,max_p,corner_p1,corner_p2 }, drawable_points_ordered;

	make_points_ordered_by_distance(biggest_rectangle, drawable_points_ordered);

	//m_cloud_viewer->update_testing_point_cloud(convex_hull_points_ordered, 255, 0, 0, 20);

	m_cloud_viewer->update_plane(drawable_points_ordered, 0, 255, 0);
}

void PickHandler::process_cylinder()
{
	if (m_cloud_viewer->m_picked_points.size() < 5)
	{
		return;
	}

	if (m_cloud_viewer->m_ic_ptr->m_cs == CS_SELECTING_POINTS)
	{
		std::cout << "please marking points represent the bottom plane of cylinder.\n";

		// green to show bottom plane choosed by user
		process_plane(m_cylinder_plane_func);

		centroid_from_points(m_cloud_viewer->m_picked_points, m_centriod_point_on_bottom);

		std::vector<point_3d> cpb{ m_centriod_point_on_bottom };

		// black color to show center point on bottom
		m_cloud_viewer->update_cylinder_centriod_point_on_bottom(cpb, 0, 0, 0, 15);

		// set the cylinder function
		m_cylinder_func.m_line_func.set_nml(m_cylinder_plane_func.a, m_cylinder_plane_func.b, m_cylinder_plane_func.c);

		m_cylinder_func.m_line_func.set_xyz(m_centriod_point_on_bottom.x, m_centriod_point_on_bottom.y, m_centriod_point_on_bottom.z);

		mean_distance_from_point_to_points(m_cloud_viewer->m_picked_points, m_centriod_point_on_bottom, m_cylinder_func.r);

		points_on_cylinder(*m_cloud_viewer->get_target_points(), m_cloud_viewer->m_cylinder_points, m_cylinder_func, m_cylinder_func.r, 0.3);

		m_cloud_viewer->update_cylinder(m_cloud_viewer->m_cylinder_points, 0, 0, 255, 4);

	}
	//else if (m_cloud_viewer->m_ic_ptr->cs_for_cylinder == CS_SELECTING_CYLINDER_DONE)
	//{
	//	std::cout << "saving points representing the cylinder. \n";

	//}

	/*
	size_t require_number = 5;

	if (m_picked_points.size() < 5)
	{
		std::cout << "points to find cylinder not enough, " << require_number - m_picked_points.size() << " points need" << std::endl;

		return;
	}
	
	std::vector<point_3d> & points = m_picked_points;

	cylinder_func _cylinder_func;

	m_cloud_viewer->m_cf.fitting_cylinder_ransac(m_picked_points, _cylinder_func, 100);

	point_3d center_p;
	float height = 0.0, radius = 0.0;
	cylinder_func_to_osg_structure(m_picked_points, _cylinder_func, center_p, height, radius);

	std::cout
		<< "center_point=" << center_p << "\n"
		<< "radius=" << radius << "\n"
		<< "height=" << height << "\n";

	Eigen::Vector3f z_axis(0, 0, 1);

	Eigen::Vector3f cylinder_line_dir(_cylinder_func.m_line_func.n, _cylinder_func.m_line_func.m, _cylinder_func.m_line_func.l);

	cylinder_line_dir.normalize();

	Eigen::Vector3f  rotated_axis = z_axis.cross(cylinder_line_dir);

	float rotated_angle = 0.0;

	angle_between_two_vector_3d(z_axis, cylinder_line_dir, rotated_angle);

	std::cout << "rotated_angle=" << rotated_angle << std::endl;

	m_cloud_viewer->update_cylinder(center_p, radius, height, rotated_axis, rotated_angle, 0, 255, 0);
	*/
}

void PickHandler::process_point()
{
	m_cloud_viewer->m_points = m_cloud_viewer->m_picked_points;
}

bool PickHandler::add_point_to_picked_vector(const point_3d & p)
{
	for (std::vector<point_3d>::iterator it = m_cloud_viewer->m_picked_points.begin();
		it != m_cloud_viewer->m_picked_points.end(); ++it)
	{
		if (it->x == p.x &&it->y == p.y &&it->z == p.z)
		{
			std::cout << "point existed" << std::endl;

			return false;
		}
	}

	m_cloud_viewer->m_picked_points.push_back(p);

	return true;
}

bool PickHandler::remove_point_from_picked_vector(const point_3d & p)
{
	for (std::vector<point_3d>::iterator it = m_cloud_viewer->m_picked_points.begin(); 
		it != m_cloud_viewer->m_picked_points.end(); ++it)
	{
		if (it->x == p.x &&it->y == p.y &&it->z == p.z)
		{
			std::cout << "point unclicked from picked set" << std::endl;

			it = m_cloud_viewer->m_picked_points.erase(it);

			return true;
		}
	}

	std::cout <<"point doesn't exist in picked set"<< std::endl;

	return false;
}
