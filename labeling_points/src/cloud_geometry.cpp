#include "cloud_geometry.h"

point_3d::point_3d(const point_3d & p)
{
	this->x = p.x;
	this->y = p.y;
	this->z = p.z;
	this->nx = p.nx;
	this->ny = p.ny;
	this->nz = p.nz;
	this->r = p.r;
	this->g = p.g;
	this->b = p.b;
}

point_3d::point_3d()
	:x(0), y(0), z(0),
	nx(0), ny(0), nz(0),
	r(0), g(0), b(0)
{
}

point_3d::point_3d(float x, float y, float z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

void point_3d::set_xyz(float x, float y, float z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

void point_3d::set_nxyz(float nx, float ny, float nz)
{
	this->nx = nx;
	this->ny = ny;
	this->nz = nz;
}

void point_3d::set_rgb(float r, float g, float b)
{
	this->r = r;
	this->g = g;
	this->b = b;
}

void point_3d::to_eigen_vector4f(Eigen::Vector4f & vector4f_p)
{
	vector4f_p(0, 0) = x;
	vector4f_p(1, 0) = y;
	vector4f_p(2, 0) = z;
	vector4f_p(3, 0) = 1;
}

void point_3d::do_transform(Eigen::Matrix4f & t, point_3d & p)
{
	Eigen::Vector4f tmp(x, y, z, 1);

	tmp = t * tmp;

	p.set_xyz(tmp(0, 0), tmp(1, 0), tmp(2, 0));
}

std::ostream & operator << (std::ostream & os, const point_3d & p)
{
	std::cout
		<< "(x,y,z,nx,ny,nz,r,g,b)="
		<< p.x << " " << p.y << " " << p.z << " "
		<< p.nx << " " << p.ny << " " << p.nz << " "
		<< p.r << " " << p.g << " " << p.b << " ";
	return os;
}

line_func_3d::line_func_3d()
	:x(0), y(0), z(0),
	n(0), m(0), l(0)
{

}

void line_func_3d::set_xyz(float x, float y, float z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

void line_func_3d::set_nml(float n, float m, float l)
{
	this->n = n;
	this->m = m;
	this->l = l;
}

plane_func_3d::plane_func_3d()
	: a(0), b(0), c(0), d(0)
{

}

void plane_func_3d::set_abcd(float a, float b, float c, float d)
{
	this->a = a;
	this->b = b;
	this->c = c;
	this->d = d;
}

cylinder_func::cylinder_func()
	: r(0)
{

}

point_3d to_point_3d(osg::Vec3d & p)
{
	return point_3d(p.x(), p.y(), p.z());
}

void convert_to_CGAL_points(std::vector<point_3d>& points, std::vector<CGAL::Simple_cartesian<float>::Point_3> & cgal_points)
{
	//cgal_points.resize(points.size());

	for (size_t i = 0; i < points.size(); ++i)
	{
		cgal_points.push_back(CGAL::Simple_cartesian<float>::Point_3(points[i].x, points[i].y, points[i].z));
	}
}

void convert_to_original_points(std::vector<CGAL::Simple_cartesian<float>::Point_3>& cgal_points, std::vector<point_3d>& points)
{
	points.resize(cgal_points.size());

	for (size_t i = 0; i < cgal_points.size(); ++i)
	{
		points[i].set_xyz(cgal_points[i].x(), cgal_points[i].y(), cgal_points[i].z());
	}
}

void convert_to_openGR_points(std::vector<point_3d>& points, std::vector<gr::Point3D<float>>& opengr_points)
{
	for (size_t i = 0; i < points.size(); ++i)
	{
		opengr_points.push_back(gr::Point3D<float>(points[i].x, points[i].y, points[i].z));
	}
}

void convert_to_pointMatcher_points(std::vector<point_3d>& points, PointMatcher<float>::DataPoints & DP)
{
	const PointMatcherIO<float>::SupportedLabels & externalLabels = PointMatcherIO<float>::getSupportedExternalLabels();

	PointMatcherIO<float>::LabelGenerator featLabelGen, descLabelGen, timeLabelGen;

	featLabelGen.add(externalLabels[0].internalName);
	featLabelGen.add(externalLabels[1].internalName);
	featLabelGen.add(externalLabels[2].internalName);
	featLabelGen.add(externalLabels[3].internalName);

	//descLabelGen.add(externalLabels[4].internalName);
	//descLabelGen.add(externalLabels[5].internalName);
	//descLabelGen.add(externalLabels[6].internalName);

	PointMatcher<float>::Matrix features = PointMatcher<float>::Matrix(4, points.size());

	//PointMatcher<float>::Matrix describe = PointMatcher<float>::Matrix(3, points.size());

	for (size_t i = 0; i < points.size(); ++i)
	{
		features(0, i) = points[i].x;
		features(1, i) = points[i].y;
		features(2, i) = points[i].z;
		features(3, i) = 1;

		//describe(0, i) = points[i].nx;
		//describe(1, i) = points[i].ny;
		//describe(2, i) = points[i].nz;
	}

	DP.features = features;
	DP.featureLabels = featLabelGen.getLabels();

	//DP.features = describe;
	//DP.featureLabels = descLabelGen.getLabels();

	if (!DP.featureExists("pad"))
	{
		DP.addFeature("pad", PointMatcher<float>::Matrix::Ones(1, features.cols()));
	}

	std::cout << std::endl;
}

void points_to_osg_structure(std::vector<point_3d>& points, osg::ref_ptr<osg::Vec3Array> coords, osg::ref_ptr<osg::Vec4Array> colors, osg::ref_ptr<osg::Vec3Array> normals, float r, float g, float b)
{
	// use point's color 
	if (r == 0 && g == 0 && b == 0)
	{
		for (size_t i = 0; i < points.size(); i++)
		{
			coords->push_back(osg::Vec3(points[i].x, points[i].y, points[i].z));

			colors->push_back(osg::Vec4(points[i].r, points[i].g, points[i].b, 1.0f));
		}
	}
	// use specific color
	else
	{
		for (size_t i = 0; i < points.size(); i++)
		{
			coords->push_back(osg::Vec3(points[i].x, points[i].y, points[i].z));
		}
		colors->push_back(osg::Vec4(r, g, b, 1.0f));
	}

	normals->push_back(osg::Vec3(0.0f, 1.0f, 0.0f));
}

void points_to_geometry_node(std::vector<point_3d>& points, osg::ref_ptr<osg::Geometry> geometry, float r, float g, float b)
{
	osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array();

	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();

	osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;

	points_to_osg_structure(points, coords, colors, normals, r, g, b);

	// use color of each point
	if (r == 0 && g == 0 && b == 0)
	{
		geometry->setColorArray(colors.get());
		geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	}
	else
	{
		geometry->setColorArray(colors.get());
		geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	}

	geometry->setVertexArray(coords.get());

	geometry->setNormalArray(normals);
	geometry->setNormalBinding(osg::Geometry::BIND_OVERALL);
}

void max_min_point_3d_vec(std::vector<point_3d>& points, point_3d & min_p, point_3d & max_p)
{
	if (points.empty())
	{
		return ;
	}

	// [0]:x [1]:y [2]:z
	std::vector<float>
		min_xyz({ points[0].x, points[1].y, points[2].z }),

		max_xyz({ points[0].x, points[1].y, points[2].z });

	//std::cout << min_xyz[0] << " " << min_xyz[1] << " " << min_xyz[2] << std::endl;

	for (size_t i = 0; i < points.size(); ++i)
	{
		point_3d & p = points[i];

		if (p.x > max_xyz[0])
		{
			max_xyz[0] = p.x;
		}
		if (p.x < min_xyz[0])
		{
			min_xyz[0] = p.x;
		}

		if (p.y > max_xyz[1])
		{
			max_xyz[1] = p.y;
		}
		if (p.y < min_xyz[1])
		{
			min_xyz[1] = p.y;
		}

		if (p.z > max_xyz[2])
		{
			max_xyz[2] = p.z;
		}
		if (p.z < min_xyz[2])
		{
			min_xyz[2] = p.z;
		}
	}

	min_p.set_xyz(min_xyz[0], min_xyz[1], min_xyz[2]);

	max_p.set_xyz(max_xyz[0], max_xyz[1], max_xyz[2]);
}

void max_min_value_array(std::vector<float> & vec, float & min_value, float & max_value)
{
	auto min_max = std::minmax_element(vec.begin(), vec.end());

	min_value = *min_max.first;

	max_value = *min_max.second;
}

void man_min_t_line_function(line_func_3d & line_func, point_3d & min_p, point_3d & max_p, std::vector<float> &min_t, std::vector<float> &max_t)
{
	min_t.resize(3, 0);

	max_t.resize(3, 0);

	min_t[0] = (min_p.x - line_func.x) / line_func.n;

	min_t[1] = (min_p.y - line_func.y) / line_func.m;

	min_t[2] = (min_p.z - line_func.z) / line_func.l;

	max_t[0] = (max_p.x - line_func.x) / line_func.n;

	max_t[1] = (max_p.y - line_func.y) / line_func.m;

	max_t[2] = (max_p.z - line_func.z) / line_func.l;
}
void get_appropriate_t(line_func_3d & line_func, std::vector<float> t_vec, point_3d target_point, float & real_t)
{
	float min_dis = FLT_MAX;

	for (size_t i = 0; i < t_vec.size(); ++i)
	{
		point_3d tmp_p;

		tmp_p.set_xyz(
			line_func.x + t_vec[i] * line_func.n,
			line_func.y + t_vec[i] * line_func.m, 
			line_func.z + t_vec[i] * line_func.l);

		float dis = 0.0;

		distance_point_to_point(tmp_p, target_point, dis);

		if (dis < min_dis)
		{
			min_dis = dis;

			real_t = t_vec[i];
		}
	}
}
void transform_points(std::vector<point_3d>& points, Eigen::Matrix4f & t, std::vector<point_3d>& ret_points)
{
	ret_points.resize(points.size());

	for (size_t i = 0; i < points.size(); ++i)
	{
		points[i].do_transform(t, ret_points[i]);
	}
}

void pedalpoint_point_to_line(const point_3d & point, const line_func_3d & _line_func_3d, point_3d & pedalpoint)
{
	float
		x0 = _line_func_3d.x,
		y0 = _line_func_3d.y,
		z0 = _line_func_3d.z,
		n = _line_func_3d.n,
		m = _line_func_3d.m,
		l = _line_func_3d.l,
		x1 = point.x,
		y1 = point.y,
		z1 = point.z;

	pedalpoint.x = (l * l * x0 + m * m * x0 + n * n * x1 - l * n*z0 - m * n*y0 + l * n*z1 + m * n*y1) / (l *l + m * m + n * n);
	pedalpoint.y = (l * l * y0 + m * m * y1 + n * n * y0 - l * m*z0 - m * n*x0 + l * m*z1 + m * n*x1) / (l *l + m * m + n * n);
	pedalpoint.z = (l * l * z1 + m * m * z0 + n * n * z0 - l * m*y0 - l * n*x0 + l * m*y1 + l * n*x1) / (l *l + m * m + n * n);
}

void pedalpoint_point_to_plane(const point_3d & point, const plane_func_3d & plane_func, point_3d & pedalpoint)
{
	float
		t = (point.x * plane_func.a + point.y * plane_func.b + point.z * plane_func.c + plane_func.d)
		/ (plane_func.a*plane_func.a + plane_func.b*plane_func.b + plane_func.c*plane_func.c);

	pedalpoint.x = point.x - plane_func.a * t;
	pedalpoint.y = point.y - plane_func.b * t;
	pedalpoint.z = point.z - plane_func.c * t;
}

void distance_points_to_line(const std::vector<point_3d>& points, const line_func_3d & _line_func_3d, std::vector<float>& points_dis_vec)
{
	points_dis_vec.resize(points.size());

	for (size_t i = 0; i < points.size(); i++)
	{
		point_3d pedal_point; float dis = 0;

		pedalpoint_point_to_line(points[i], _line_func_3d, pedal_point);

		distance_point_to_point(points[i], pedal_point, dis);

		points_dis_vec[i] = dis;
	}
}

void distance_point_to_point(const point_3d & point_1, const point_3d & point_2, float & distance)
{
	distance = sqrt(
		(point_1.x - point_2.x)*(point_1.x - point_2.x) +
		(point_1.y - point_2.y)*(point_1.y - point_2.y) +
		(point_1.z - point_2.z)*(point_1.z - point_2.z));
}

void save_points(const std::vector<point_3d>& points, const std::string & filename)
{
	std::ofstream of(filename, std::ios::out);

	if (filename.find(".txt") != std::string::npos)
	{
		for (size_t i = 0; i < points.size(); ++i)
		{
			of
				<< points[i].x << " " << points[i].y << " " << points[i].z << " "

				<< points[i].nx << " " << points[i].ny << " " << points[i].nz << " "

				<< points[i].r << " " << points[i].g << " " << points[i].b

				<< std::endl;
		}

		of.close();
	}
}

void make_points_ordered_by_distance(std::vector<point_3d>& points, std::vector<point_3d>& ordered_points)
{
	if (points.empty())
	{
		return;
	}

	std::vector<bool> visited(points.size(), false);

	ordered_points.push_back(points[0]);

	visited[0] = true;

	for (size_t i = 0; i < ordered_points.size(); ++i)
	{
		float min_dis = FLT_MAX;

		int min_dis_i = -1;

		for (size_t j = 0; j < points.size(); ++j)
		{
			if (visited[j] == false)
			{
				float dis = 0.0;

				distance_point_to_point(ordered_points[i], points[j], dis);

				if (dis < min_dis)
				{
					min_dis_i = j;

					min_dis = dis;
				}
			}
		}

		if (min_dis_i != -1)
		{
			visited[min_dis_i] = true;

			ordered_points.push_back(points[min_dis_i]);
		}
	}

	//for (size_t i = 0; i < ordered_points.size() - 1; i++)
	//{
	//	float dis = 0.0;

	//	distance_point_to_point(ordered_points[i], ordered_points[i + 1], dis);

	//	std::cout << dis << " ";
	//}
	//std::cout << std::endl;
}

void point_along_with_vector_within_dis(point_3d & point, Eigen::Vector3f & line_dir, point_3d & result_p1, point_3d & result_p2, float distance)
{
	if (distance == 0)
	{
		result_p1 = point;

		result_p2 = point;
		return;
	}
	float m, n, l;
	m = line_dir[0];
	n = line_dir[1];
	l = line_dir[2];

	float
		M = sqrt((distance*distance*m*m) / (m*m + n * n + l * l)),
		N = sqrt((distance*distance*n*n) / (m*m + n * n + l * l)),
		L = sqrt((distance*distance*l*l) / (m*m + n * n + l * l));

	float x[2], y[2], z[2];

	x[0] = point.x + M;
	x[1] = point.x - M;
	y[0] = point.y + N;
	y[1] = point.y - N;
	z[0] = point.z + L;
	z[1] = point.z - L;

	Eigen::Vector3f v;

	std::vector<point_3d> tmp_p;

	//! 筛选出两个平行于方向向量的点
	for (size_t i = 0; i < 2; i++)
	{
		for (size_t j = 0; j < 2; j++)
		{
			for (size_t k = 0; k < 2; k++)
			{
				v[0] = x[i] - point.x;
				v[1] = y[j] - point.y;
				v[2] = z[k] - point.z;
				if (is_parallel_vector(line_dir, v))
				{
					tmp_p.push_back(point_3d(x[i], y[j], z[k]));
				}							 
			}								 
		}
	}
	if (tmp_p.size() > 1)
	{
		result_p1 = tmp_p[0];
		result_p2 = tmp_p[1];
	}
}

bool is_parallel_vector(const Eigen::Vector3f & v1, const Eigen::Vector3f & v2)
{
	//std::cout << v1.cross(v2).norm() << std::endl;
	if (v1.cross(v2).norm() < 0.01)
	{
		return true;
	}
	return false;
}

void point_cloud::load_points(std::vector<point_3d> & points)
{
	this->pts = points;
}