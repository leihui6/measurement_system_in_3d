#include "cloud_viewer.h"

cloud_viewer::cloud_viewer(const std::string & window_name)
	: m_root(new osg::Group()),
	m_viewer(new osgViewer::Viewer)
{
	m_viewer->addEventHandler(new osgGA::StateSetManipulator(m_viewer->getCamera()->getOrCreateStateSet()));

	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	traits->x = 40;
	traits->y = 40;
	traits->width = 600;
	traits->height = 480;
	traits->windowName = window_name;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());

	osg::ref_ptr<osg::Camera> camera = new osg::Camera;
	camera->setGraphicsContext(gc.get());
	camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
	//GLenum buffer = (traits->doubleBuffer) ? GL_BACK : GL_FRONT;
	//camera->setDrawBuffer(buffer);
	//camera->setReadBuffer(buffer);

	m_viewer->addSlave(camera.get());
}

cloud_viewer::~cloud_viewer()
{
}

void cloud_viewer::add_point_cloud(std::vector<point_3d> & points, Eigen::Matrix4f t)
{
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();

	osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array();

	osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array();

	osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;

	normals->push_back(osg::Vec3(0.0f, 1.0f, 0.0f));

	for (size_t i = 0; i < points.size(); i++)
	{
		coords->push_back(osg::Vec3(points[i].x, points[i].y, points[i].z));

		color->push_back(osg::Vec4(points[i].r, points[i].g, points[i].b, 1.0f));
	}

	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();

	geometry->setVertexArray(coords.get());

	geometry->setColorArray(color.get());
	geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

	geometry->setNormalArray(normals);
	geometry->setNormalBinding(osg::Geometry::BIND_OVERALL);

	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, points.size()));

	geode->addDrawable(geometry.get());

	osg::ref_ptr<osg::MatrixTransform> transformation = new osg::MatrixTransform;

	t.transposeInPlace();

	transformation->setMatrix(osg::Matrixf(
		t(0, 0), t(0, 1), t(0, 2), t(0, 3),
		t(1, 0), t(1, 1), t(1, 2), t(1, 3),
		t(2, 0), t(2, 1), t(2, 2), t(2, 3),
		t(3, 0), t(3, 1), t(3, 2), t(3, 3)));

	transformation->addChild(geode.get());

	m_root->addChild(transformation.get());

	// 优化场景数据
	//osgUtil::Optimizer optimizer;
	//optimizer.optimize(m_root.get());
	//m_viewer->setSceneData(m_root.get());

	// 窗口大小变化事件
	//m_viewer->addEventHandler(new osgViewer::WindowSizeHandler);
}

void cloud_viewer::display()
{
	std::cout << m_root->getNumChildren() << std::endl;

	m_viewer->setSceneData(m_root.get());
	//m_viewer->realize();
	m_viewer->run();
}
