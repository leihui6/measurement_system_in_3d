#include "cloud_geometry.h"

point_3d::point_3d(const point_3d & p)
{
	this->set_xyz(p.x, p.y, p.z);
	this->set_nxyz(p.nx, p.ny, p.nz);
	this->set_rgb(p.r, p.g, p.b);
}

point_3d::point_3d()
	:x(0), y(0), z(0),
	nx(0), ny(0), nz(0),
	r(0), g(0), b(0)
{
}

point_3d::point_3d(float x, float y, float z)
{
	this->set_xyz(x, y, z);
	this->set_nxyz(0, 0, 0);
	this->set_rgb(0, 0, 0);
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

Eigen::Vector3f point_3d::get_vector3f()
{
	return Eigen::Vector3f(this->x, this->y, this->z);
}

void point_3d::do_transform(Eigen::Matrix4f & t, point_3d & p)
{
	Eigen::Vector4f tmp(x, y, z, 1);

	tmp = t * tmp;

	p.set_xyz(tmp(0, 0), tmp(1, 0), tmp(2, 0));
}

void point_3d::do_transform(Eigen::Matrix4f & t)
{
	Eigen::Vector4f tmp(x, y, z, 1);

	tmp = t * tmp;

	this->set_xyz(tmp(0, 0), tmp(1, 0), tmp(2, 0));
}

point_3d & point_3d::operator=(const point_3d & p)
{
	if (this != &p)
	{
		this->set_xyz(p.x, p.y, p.z);
		this->set_nxyz(p.nx, p.ny, p.nz);
		this->set_rgb(p.r, p.g, p.b);
	}
	return *this;
}

point_3d point_3d::operator+(const point_3d & p)
{
	point_3d tp;

	tp.set_xyz(this->x + p.x, this->y + p.y, this->z + p.z);

	return tp;
}

point_3d point_3d::operator/(const float num)
{
	point_3d tp;

	tp.set_xyz(this->x / num, this->y / num, this->z / num);

	return tp;
}

bool point_3d::operator==(const point_3d & rp)
{
	if (this->x == rp.x &&
		this->y == rp.y &&
		this->z == rp.z)
		return true;
	return false;
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
	:origin(),
	direction()
{

}

void line_func_3d::set_xyz(float x, float y, float z)
{
	this->origin[0] = x;
	this->origin[1] = y;
	this->origin[2] = z;
}

void line_func_3d::set_nml(float n, float m, float l)
{
	this->direction[0] = n;
	this->direction[1] = m;
	this->direction[2] = l;
}

point_3d line_func_3d::get_origin_point_3d()
{
	return point_3d(origin[0], origin[1], origin[2]);
}

point_3d line_func_3d::get_direction_point_3d()
{
	return point_3d(direction[0], direction[1], direction[2]);
}

//Eigen::Vector3f line_func_3d::direction()
//{
//	return Eigen::Vector3f(this->n, this->m, this->l);
//}
//
//Eigen::Vector3f line_func_3d::origin_point()
//{
//	return Eigen::Vector3f(this->x, this->y, this->z);
//}

plane_func_3d::plane_func_3d()
	: a(0), b(0), c(0), d(0)
{

}

void plane_func_3d::set_abcd(float _a, float _b, float _c, float _d)
{
	a = _a;
	b = _b;
	c = _c;
	d = _d;
}

void plane_func_3d::set_abcd(float _a, float _b, float _c, point_3d & p)
{
	a = _a;
	b = _b;
	c = _c;
	d = -(a*p.x + b * p.y + c * p.z);
}


//Eigen::Vector3f plane_func_3d::direction()
//{
//	return Eigen::Vector3f(this->a, this->b, this->c);
//}

cylinder_func::cylinder_func()
	: radius(0),
	height(0),
	axis()
{

}

point_3d to_point_3d(osg::Vec3d & p)
{
	return point_3d(p.x(), p.y(), p.z());
}

void convert_to_CGAL_points(std::vector<point_3d> & points, std::vector<std::pair<Kernel::Point_3, Kernel::Vector_3>> & cgal_points)
{
	for (size_t i = 0; i < points.size(); ++i)
	{
		cgal_points.push_back(
			std::make_pair<Kernel::Point_3, Kernel::Vector_3>(
				Kernel::Point_3(points[i].x, points[i].y, points[i].z),
				Kernel::Vector_3(points[i].nx, points[i].ny, points[i].nz)
				));
			//CGAL::Simple_cartesian<float>::Point_3(points[i].x, points[i].y, points[i].z));
	}
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

void points_to_osg_structure(std::vector<point_3d>& points, osg::ref_ptr<osg::Vec3Array> coords, osg::ref_ptr<osg::Vec4Array> colors, osg::ref_ptr<osg::Vec3Array> normals, float r, float g, float b, float w)
{
	// use point's color 
	if (r == 0 && g == 0 && b == 0)
	{
		for (size_t i = 0; i < points.size(); i++)
		{
			coords->push_back(osg::Vec3(points[i].x, points[i].y, points[i].z));

			colors->push_back(osg::Vec4(points[i].r, points[i].g, points[i].b, w));
		}
	}
	// use specific color
	else
	{
		for (size_t i = 0; i < points.size(); i++)
		{
			coords->push_back(osg::Vec3(points[i].x, points[i].y, points[i].z));
		}
		colors->push_back(osg::Vec4(r, g, b, w));
	}

	normals->push_back(osg::Vec3(0.0f, 1.0f, 0.0f));
}

void cylinder_func_to_osg_structure(std::vector<point_3d>& points, cylinder_func & cl, point_3d & center_p, float &height, float  &radius)
{
	size_t max_distance_i = 0;

	float max_distance = FLT_MIN;

	for (size_t i = 0; i < points.size(); ++i)
	{
		point_3d pedal_point;

		pedalpoint_point_to_line(points[i], cl.axis, pedal_point);

		float dis = 0.0;

		distance_point_to_point(pedal_point, cl.axis.get_origin_point_3d(), dis);

		if (dis > max_distance)
		{
			max_distance = dis;

			height = dis;
		}
	}

	center_p = cl.axis.get_origin_point_3d();

	radius = cl.radius;
}

void points_to_geometry_node(std::vector<point_3d>& points, osg::ref_ptr<osg::Geometry> geometry, float r, float g, float b, float w)
{
	osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array();

	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();

	osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;

	points_to_osg_structure(points, coords, colors, normals, r, g, b, w);

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

//void man_min_t_line_function(line_func_3d & line_func, point_3d & min_p, point_3d & max_p, std::vector<float> &min_t, std::vector<float> &max_t)
//{
//	min_t.resize(3, 0);
//
//	max_t.resize(3, 0);
//
//	min_t[0] = (min_p.x - line_func.x) / line_func.n;
//
//	min_t[1] = (min_p.y - line_func.y) / line_func.m;
//
//	min_t[2] = (min_p.z - line_func.z) / line_func.l;
//
//	max_t[0] = (max_p.x - line_func.x) / line_func.n;
//
//	max_t[1] = (max_p.y - line_func.y) / line_func.m;
//
//	max_t[2] = (max_p.z - line_func.z) / line_func.l;
//}
//void get_appropriate_t(line_func_3d & line_func, std::vector<float> t_vec, point_3d target_point, float & real_t)
//{
//	float min_dis = FLT_MAX;
//
//	for (size_t i = 0; i < t_vec.size(); ++i)
//	{
//		point_3d tmp_p;
//
//		tmp_p.set_xyz(
//			line_func.x + t_vec[i] * line_func.n,
//			line_func.y + t_vec[i] * line_func.m, 
//			line_func.z + t_vec[i] * line_func.l);
//
//		float dis = 0.0;
//
//		distance_point_to_point(tmp_p, target_point, dis);
//
//		if (dis < min_dis)
//		{
//			min_dis = dis;
//
//			real_t = t_vec[i];
//		}
//	}
//}
void transform_points(std::vector<point_3d>& points, Eigen::Matrix4f & t, std::vector<point_3d>& ret_points)
{
	if (&points == &ret_points)
	{
		std::vector<point_3d> points_tmp(points.size());;

		for (size_t i = 0; i < points.size(); ++i)
		{
			points[i].do_transform(t, points_tmp[i]);
		}

		ret_points = points_tmp;
	}
	else
	{
		ret_points.resize(points.size());

		for (size_t i = 0; i < points.size(); ++i)
		{
			points[i].do_transform(t, ret_points[i]);
		}
	}
}

void pedalpoint_point_to_line(const point_3d & point, const line_func_3d & _line_func_3d, point_3d & pedalpoint)
{
	float
		x0 = _line_func_3d.origin[0],
		y0 = _line_func_3d.origin[1],
		z0 = _line_func_3d.origin[2],
		n = _line_func_3d.direction[0],
		m = _line_func_3d.direction[1],
		l = _line_func_3d.direction[2],
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
		float dis = 0;

		distance_point_to_line(points[i], _line_func_3d, dis);

		points_dis_vec[i] = dis;
	}
}

void distance_point_to_line(const point_3d & point, const line_func_3d & _line_func_3d, float & points_dis)
{
	point_3d pedal_point;

	pedalpoint_point_to_line(point, _line_func_3d, pedal_point);

	distance_point_to_point(point, pedal_point, points_dis);
}

void distance_point_to_point(const point_3d & point_1, const point_3d & point_2, float & distance)
{
	distance = sqrt(
		(point_1.x - point_2.x)*(point_1.x - point_2.x) +
		(point_1.y - point_2.y)*(point_1.y - point_2.y) +
		(point_1.z - point_2.z)*(point_1.z - point_2.z));
}

void distance_point_to_plane(const point_3d & point, const plane_func_3d & plane_func, float & distance)
{
	point_3d pedal_point;

	pedalpoint_point_to_plane(point, plane_func, pedal_point);

	distance_point_to_point(point, pedal_point, distance);
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
	double m, n, l;
	m = line_dir[0];
	n = line_dir[1];
	l = line_dir[2];

	double
		M = sqrt((distance*distance*m*m) / (m*m + n * n + l * l)),
		N = sqrt((distance*distance*n*n) / (m*m + n * n + l * l)),
		L = sqrt((distance*distance*l*l) / (m*m + n * n + l * l));

	double x[2], y[2], z[2];

	x[0] = point.x + M;
	x[1] = point.x - M;
	y[0] = point.y + N;
	y[1] = point.y - N;
	z[0] = point.z + L;
	z[1] = point.z - L;

	Eigen::Vector3f v;

	std::vector<point_3d> tmp_p;
	line_dir.normalize();

	double std = 1.0f;

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

				v.normalize();

				double res = abs((double)line_dir.dot(v));
				if (equal_float(std, res))
					tmp_p.push_back(point_3d(x[i], y[j], z[k]));
				if (equal_float(x[0], x[1])) ++i;
				if (equal_float(y[0], y[1])) ++j;
				if (equal_float(z[0], z[1])) ++k;
			}
		}
	}
	// std::cout << "----------" << std::endl;
	if (tmp_p.size() == 2)
	{
		result_p1 = tmp_p[0];
		result_p2 = tmp_p[1];
	}
	else if (tmp_p.size() > 2)
	{
		std::cout << "[warning] point_along_with_vector_within_dis" << std::endl;
		result_p1 = tmp_p[0];
		result_p2 = tmp_p[1];
	}
	else
	{
		std::cout << "[error] point_along_with_vector_within_dis" << std::endl;
	}
}

void point_along_with_vector_within_dis(point_3d & point, Eigen::Vector3f & line_dir, point_3d & result_p, float distance)
{
	point_3d p1, p2;

	point_along_with_vector_within_dis(point, line_dir, p1, p2, distance);

	Eigen::Vector3f v1, v2;
	v1[0] = p1.x - point.x;
	v1[1] = p1.y - point.y;
	v1[2] = p1.z - point.z;

	v2[0] = p2.x - point.x;
	v2[1] = p2.y - point.y;
	v2[2] = p2.z - point.z;


	if (v1.dot(line_dir) > 0)
		result_p.set_xyz(p1.x, p1.y, p1.z);
	else if (v1.dot(line_dir) > 0)
		result_p.set_xyz(p2.x, p2.y, p2.z);
	else
		result_p.set_xyz(p2.x, p2.y, p2.z);
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

void plane_function_from_three_points(point_3d & A, point_3d & B, point_3d & C, plane_func_3d & plane_func)
{
	Eigen::Vector3f 
		AC = Eigen::Vector3f(C.x - A.x, C.y - A.y, C.z - A.z),
		AB = Eigen::Vector3f(B.x - A.x, B.y - A.y, B.z - A.z);

	Eigen::Vector3f
		N = AC.cross(AB);
	
	float d = -(N[0] * A.x + N[1] * A.y + N[2] * A.z);

	plane_func.set_abcd(N[0], N[1], N[2], d);
}

void intersection_line_to_sphere(line_func_3d & line_func, point_3d & sphere_center, float & sphere_r, point_3d & res_p1, point_3d & res_p2)
{
	point_3d pedal_point_on_line;

	pedalpoint_point_to_line(sphere_center, line_func, pedal_point_on_line);

	float distance_to_center;

	distance_point_to_point(pedal_point_on_line, sphere_center, distance_to_center);

	if (distance_to_center > sphere_r)
	{
		return;
	}

	float distance_on_line = sqrt(sphere_r * sphere_r - distance_to_center * distance_to_center);

	point_along_with_vector_within_dis(pedal_point_on_line, line_func.direction, res_p1, res_p2, distance_on_line);
}

void points_on_plane(std::vector<point_3d>& points, std::vector<point_3d>& points_on_plane, plane_func_3d & plane_func, float distance_threshold)
{
	points_on_plane.clear();

	for (size_t i = 0; i < points.size(); i++)
	{
		float dis_to_plane = 0.0;

		distance_point_to_plane(points[i], plane_func, dis_to_plane);

		if (dis_to_plane < distance_threshold)
		{
			points_on_plane.push_back(points[i]);
		}
	}
}

void points_on_line(std::vector<point_3d>& points, std::vector<point_3d>& points_on_line, line_func_3d & line_func, float distance_threshold)
{
	points_on_line.clear();

	for (size_t i = 0; i < points.size(); i++)
	{
		float dis_to_line = 0.0;

		distance_point_to_line(points[i], line_func, dis_to_line);

		if (dis_to_line < distance_threshold)
		{
			points_on_line.push_back(points[i]);
		}
	}
}

void points_line(line_func_3d & line_func, std::vector<point_3d>& points, std::vector<point_3d>& points_line)
{
	point_3d p_A, p_B;
	segment_point_from_points(p_A, p_B, points, line_func);

	// Make sure the direction of p_A and p_B is the same as $line_func
	Eigen::Vector3f v, line_direction;
	v[0] = p_B.x - p_A.x;
	v[1] = p_B.y - p_A.y;
	v[2] = p_B.z - p_A.z;
	line_direction = line_func.direction;
	if (v.dot(line_direction) < 0)
	{
		line_direction = -1 * line_direction;
	}
	float line_distance;
	distance_point_to_point(p_A, p_B, line_distance);
	float step_number = 50;
	float step_forward = line_distance / step_number;

	point_3d begin_p = p_A;
	for (float i = 0; i < step_number; ++i)
	{
		point_3d result_p;
		point_along_with_vector_within_dis(begin_p, line_direction, result_p, step_forward);
		begin_p = result_p;

		points_line.push_back(result_p);
	}
}

void points_plane(plane_func_3d & plane_func, std::vector<point_3d>& points, std::vector<point_3d>& points_plane)
{
	point_3d max_p, min_p;
	max_min_point_3d_vec(points, min_p, max_p);

	float A, B, C, D;
	A = plane_func.a;
	B = plane_func.b;
	C = plane_func.c;
	D = plane_func.d;
	float 
		step_number = 20,
		in_plane_threshold = 0.1;

	float
		x_step = (max_p.x - min_p.x) / step_number,
		y_step = (max_p.y - min_p.y) / step_number,
		z_step = (max_p.z - min_p.z) / step_number;

	for (float x = min_p.x; x < max_p.x; x += x_step)
		for (float y = min_p.y; y < max_p.y; y += y_step)
			for (float z = min_p.z; z < max_p.z; z += z_step)
				if (abs(A * x + B * y + C * z + D) < in_plane_threshold)
					points_plane.push_back(point_3d(x, y, z));
	std::cout << "total:" << points_plane.size() << std::endl;
}

void segment_point_from_points(point_3d & p_A, point_3d & p_B, std::vector<point_3d>& points, line_func_3d & line_func)
{
	std::vector<point_3d> pedalpoint_vec;
	pedalpoint_vec.resize(points.size());
	for (size_t i = 0; i < points.size(); ++i)
		pedalpoint_point_to_line(points[i], line_func, pedalpoint_vec[i]);

	point_3d line_center;
	centroid_from_points(pedalpoint_vec, line_center);
	
	Eigen::Vector3f line_direction = line_func.direction;
	float max_dis[2] = { FLT_MIN, FLT_MIN };

	for (size_t i = 0; i < pedalpoint_vec.size(); ++i)
	{
		Eigen::Vector3f v;
		v[0] = pedalpoint_vec[i].x - line_center.x;
		v[1] = pedalpoint_vec[i].y - line_center.y;
		v[2] = pedalpoint_vec[i].z - line_center.z;

		float distance = 0;
		distance_point_to_point(pedalpoint_vec[i], line_center, distance);

		// positive direction
		if (line_direction.dot(v) > 0)
		{
			if (distance > max_dis[0])
			{
				max_dis[0] = distance;
				p_A = pedalpoint_vec[i];
			}
		}
		else
		{
			if (distance > max_dis[1])
			{
				max_dis[1] = distance;
				p_B = pedalpoint_vec[i];
			}
		}
	}
}

void points_on_cylinder(std::vector<point_3d>& points, std::vector<point_3d>& points_on_cylinder, cylinder_func & _cylinder_func, float threshold)
{
	points_on_cylinder.clear();

	//std::cout << points.size() << std::endl;

	for (size_t i = 0; i < points.size(); i++)
	{
		float dis_to_line = 0.0;

		distance_point_to_line(points[i], _cylinder_func.axis, dis_to_line);

		if (fabs(dis_to_line - _cylinder_func.radius) < threshold)
		{
			points_on_cylinder.push_back(points[i]);
		}
	}
}

void points_on_plane_circle(std::vector<point_3d>& points, std::vector<point_3d>& points_on_plane_circle, plane_func_3d & plane_func, point_3d & circle_center, float circle_r, float threshold_on_plane, float threshold_in_circle)
{
	for (size_t i = 0; i < points.size(); ++i)
	{
		float on_plane, in_circle;

		distance_point_to_plane(points[i], plane_func, on_plane);

		distance_point_to_point(points[i], circle_center, in_circle);

		in_circle = fabs(in_circle - circle_r);

		if ((in_circle < threshold_in_circle) && (on_plane < threshold_on_plane))
			points_on_plane_circle.push_back(points[i]);
	}
}

void centroid_from_points(std::vector<point_3d>& points, point_3d & centroid_point)
{
	float sum_xyz[3] = { 0,0,0 };

	for (size_t i = 0; i < points.size(); ++i)
	{
		sum_xyz[0] += points[i].x;

		sum_xyz[1] += points[i].y;
		
		sum_xyz[2] += points[i].z;
	}

	centroid_point.set_xyz(sum_xyz[0] / points.size(), sum_xyz[1] / points.size(), sum_xyz[2] / points.size());

	//std::cout << "centroid_point:" << centroid_point << std::endl;
}

void standard_deviation(std::vector<float>& vec, float & deviation)
{
	if (vec.size() < 2)
	{
		deviation = 0.0;

		return; 
	}

	float sum = std::accumulate(vec.begin(), vec.end(), 0);

	float mean_value = sum / vec.size();

	sum = 0;

	for (size_t i = 0; i < vec.size(); ++i)
	{
		sum += powf(vec[i] - mean_value, 2);
	}

	deviation = sqrtf(sum / (vec.size() - 1));
}

void probability_close_to_value(std::vector<float>& vec, float specifical_value, float threshold, float & probability)
{
	size_t above_count = 0;

	for (size_t i = 0; i < vec.size(); i++)
	{
		if (fabs(vec[i] - specifical_value) < threshold)
		{
			above_count++;
		}
	}

	probability = above_count / (float)vec.size();
}

void mean_distance_from_point_to_points(std::vector<point_3d>& points, point_3d & point, float & mean_distance)
{
	float sum = 0.0;

	for (size_t i = 0; i < points.size(); ++i)
	{
		float dis = 0.0;

		distance_point_to_point(points[i], point, dis);

		sum += dis;
	}

	mean_distance = (sum / points.size());
}

void longgest_distance_from_point_to_points(std::vector<point_3d>& points, point_3d & point, float & longgest_distance)
{
	longgest_distance = FLT_MIN;

	for (size_t i = 0; i < points.size(); ++i)
	{
		float dis = 0.0;

		distance_point_to_point(points[i], point, dis);

		if (dis > longgest_distance)
		{
			longgest_distance = dis;
		}
	}
}

void angle_between_two_vector_3d(point_3d p1, point_3d p2, float & angle)
{
	Eigen::Vector3f ep1(p1.x, p1.y, p1.z), ep2(p2.x, p2.y, p2.z);

	float
		dot_value = ep1.dot(ep2),
		lenSq1 = ep1.norm(),
		lenSq2 = ep2.norm();

	angle = acos(dot_value / sqrt(lenSq1 * lenSq2));
}

bool is_in_range_of_two_points(point_3d & p, point_3d & p1, point_3d & p2)
{
	float
		x[2] = { p1.x,p2.x },
		y[2] = { p1.y,p2.y },
		z[2] = { p1.z,p2.z };

	if (p1.x > p2.x)
	{
		x[0] = p2.x;
		x[1] = p1.x;
	}
	if (p1.y > p2.y)
	{
		y[0] = p2.y;
		y[1] = p1.y;
	}
	if (p1.z > p2.z)
	{
		z[0] = p2.z;
		z[1] = p1.z;
	}

	if (
		p.x < x[1] && p.x > x[0] &&
		p.y < y[1] && p.y > y[0] &&
		p.z < z[1] && p.z > z[0])
	{
		return true;
	}

	return false;
}

bool equal_float(double v1, double v2, double th)
{
	if (abs(v1 - v2) < th)
		return true;
	return false;
}

void angle_between_two_vector_3d(Eigen::Vector3f p1, Eigen::Vector3f p2, float & angle)
{
	float
		dot_value = p1.dot(p2),
		lenSq1 = p1.norm(),
		lenSq2 = p2.norm();

	angle = acosf(dot_value / (lenSq1 * lenSq2))* 180.0 / M_PI;
}

void point_cloud::load_points(std::vector<point_3d> & points)
{
	this->pts = points;
}