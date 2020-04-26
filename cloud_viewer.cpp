#include "cloud_viewer.h"


bool PickHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (ea.getEventType() != osgGA::GUIEventAdapter::RELEASE ||
		ea.getButton() != osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON ||
		!(ea.getModKeyMask()&osgGA::GUIEventAdapter::MODKEY_CTRL))
		return false;

	osgViewer::View* viewer = dynamic_cast<osgViewer::View*>(&aa);
	if (viewer)
	{
		std::cout << ea.getX() << ea.getY() << std::endl;

		osg::Vec3 window(ea.getX(), ea.getY(), 0);

		osg::Vec3 object;

		osg::ref_ptr<osg::Camera> camera = viewer->getCamera();
		//osg::Vec3d vScreen(x,y, 0);
		osg::Matrix mVPW = camera->getViewMatrix() * camera->getProjectionMatrix();

		//if (camera->getViewport())
		//{
			mVPW = mVPW * camera->getViewport()->computeWindowMatrix();
		//}

		osg::Matrix invertVPW;
		invertVPW.invert(mVPW);
		object = window * invertVPW;

		std::cout << object.x() << " " << object.y() << " " << object.z() << " " << std::endl;

		point_3d one_point;

		one_point.set_xyz(object.x(), object.y(), object.z());

		points_test.push_back(one_point);

		m_cloud_viewer->add_point_cloud_with_color(points_test, Eigen::Matrix4f::Identity(), 255, 255, 255);

		if (points_test.size() == 10)
		{
			save_points(points_test, "data/test_pick_points.txt");
		}

		//osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector =
		//	new osgUtil::LineSegmentIntersector(osgUtil::Intersector::WINDOW, ea.getX(), ea.getY());
		//osgUtil::IntersectionVisitor iv(intersector.get());
		//viewer->getCamera()->accept(iv);

		//if (intersector->containsIntersections())
		//{
		//	osgUtil::LineSegmentIntersector::Intersection result = *(intersector->getIntersections().begin());
		//	
		//	doUserOperations(result);
		//}
	}
	return false;
}

void PickHandler::doUserOperations(osgUtil::LineSegmentIntersector::Intersection& result)
{
	// test
	//std::ofstream test("test.txt");
	//test << __TIME__ << std::endl;
	//test.close();

}

cloud_viewer::cloud_viewer(const std::string & window_name)
	: m_root(new osg::Group()),
	m_viewer(new osgViewer::Viewer)
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

	//m_viewer->addSlave(camera.get());
}

cloud_viewer::~cloud_viewer()
{

}

void cloud_viewer::add_point_cloud_with_color(std::vector<point_3d> & points, Eigen::Matrix4f t, float r, float g, float b)
{
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();

	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();

	points_to_geometry_node(points, geometry, r, g, b);

	geode->addDrawable(geometry);

	osg::ref_ptr<osg::MatrixTransform> transformation = new osg::MatrixTransform;

	t.transposeInPlace();

	transformation->setMatrix(osg::Matrixf(
		t(0, 0), t(0, 1), t(0, 2), t(0, 3),
		t(1, 0), t(1, 1), t(1, 2), t(1, 3),
		t(2, 0), t(2, 1), t(2, 2), t(2, 3),
		t(3, 0), t(3, 1), t(3, 2), t(3, 3)));

	m_root->addChild(transformation.get());

	m_root->addChild(geode.get());

	m_viewer->run();
}

void cloud_viewer::add_point_cloud(std::vector<point_3d> & points, Eigen::Matrix4f t)
{
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();

	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();

	points_to_geometry_node(points, geometry);

	geode->addDrawable(geometry);

	osg::ref_ptr<osg::MatrixTransform> transformation = new osg::MatrixTransform;

	t.transposeInPlace();

	transformation->setMatrix(osg::Matrixf(
		t(0, 0), t(0, 1), t(0, 2), t(0, 3),
		t(1, 0), t(1, 1), t(1, 2), t(1, 3),
		t(2, 0), t(2, 1), t(2, 2), t(2, 3),
		t(3, 0), t(3, 1), t(3, 2), t(3, 3)));

	m_root->addChild(transformation.get());

	m_root->addChild(geode.get());

	//m_viewer->run();
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

void cloud_viewer::add_test_points()
{
	const osg::Vec4 normalColor(1.0f, 1.0f, 1.0f, 1.0f);

	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(8);
	(*vertices)[0].set(-0.5f, -0.5f, -0.5f);
	(*vertices)[1].set(0.5f, -0.5f, -0.5f);
	(*vertices)[2].set(0.5f, 0.5f, -0.5f);
	(*vertices)[3].set(-0.5f, 0.5f, -0.5f);
	(*vertices)[4].set(-0.5f, -0.5f, 0.5f);
	(*vertices)[5].set(0.5f, -0.5f, 0.5f);
	(*vertices)[6].set(0.5f, 0.5f, 0.5f);
	(*vertices)[7].set(-0.5f, 0.5f, 0.5f);

	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array(1);
	(*colors)[0] = normalColor;

	osg::ref_ptr<osg::DrawElementsUInt> indices = new osg::DrawElementsUInt(GL_QUADS, 24);
	(*indices)[0] = 0; (*indices)[1] = 1; (*indices)[2] = 2; (*indices)[3] = 3;
	(*indices)[4] = 4; (*indices)[5] = 5; (*indices)[6] = 6; (*indices)[7] = 7;
	(*indices)[8] = 0; (*indices)[9] = 1; (*indices)[10] = 5; (*indices)[11] = 4;
	(*indices)[12] = 1; (*indices)[13] = 2; (*indices)[14] = 6; (*indices)[15] = 5;
	(*indices)[16] = 2; (*indices)[17] = 3; (*indices)[18] = 7; (*indices)[19] = 6;
	(*indices)[20] = 3; (*indices)[21] = 0; (*indices)[22] = 4; (*indices)[23] = 7;

	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	geom->setDataVariance(osg::Object::DYNAMIC);
	geom->setUseDisplayList(false);
	geom->setUseVertexBufferObjects(true);
	geom->setVertexArray(vertices.get());
	geom->setColorArray(colors.get());
	geom->setColorBinding(osg::Geometry::BIND_OVERALL);
	geom->addPrimitiveSet(indices.get());

	m_root->addChild(geom);
}

void cloud_viewer::display()
{
	std::cout << "display()" << m_root->getNumChildren() << std::endl;;

	osg::ref_ptr<PickHandler> selector = new PickHandler(this);

	m_viewer->addEventHandler(selector.get());

	m_viewer->setSceneData(m_root.get());

	//osgUtil::Optimizer optimizer;
	//optimizer.optimize(m_root.get());
	
	m_viewer->realize();
	
	m_viewer->run();
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

	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, points.size()));

	osg::StateSet* stateSet = geometry->getOrCreateStateSet();
	osg::Point* state_point_size = new osg::Point;
	state_point_size->setSize(4.0);
	stateSet->setAttribute(state_point_size);
}

