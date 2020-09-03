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

std::ostream & operator << (std::ostream & os, const point_3d & p)
{
	std::cout
		<< "(x,y,z,nx,ny,nz,r,g,b)="
		<< p.x << " " << p.y << " " << p.z << " "
		<< p.nx << " " << p.ny << " " << p.nz << " "
		<< p.r << " " << p.g << " " << p.b << " ";
	return os;
}

void points_to_geometry_node(std::vector<point_3d>& points, osg::ref_ptr<osg::Geometry> geometry, float w)
{
	osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array();
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
	points_to_osg_structure(points, coords, colors, w);

	// vertex
	geometry->setVertexArray(coords.get());

	// color
	geometry->setColorArray(colors.get());
	geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

	// normals
	//osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
	//normals->push_back(osg::Vec3(1.0f, 0.0f, 0.0f));
	//geometry->setNormalArray(normals);
	//geometry->setNormalBinding(osg::Geometry::BIND_OVERALL);

	//geometry->getOrCreateStateSet()->setMode(GL_BLEND,osg::StateAttribute::ON);
	//geometry->getOrCreateStateSet()->setMode(GL_DEPTH_TEST,osg::StateAttribute::ON);
	geometry->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
}

void points_to_osg_structure(std::vector<point_3d>& points, osg::ref_ptr<osg::Vec3Array> coords, osg::ref_ptr<osg::Vec4Array> colors, float w)
{
	for (size_t i = 0; i < points.size(); i++)
	{
		coords->push_back(osg::Vec3(points[i].x, points[i].y, points[i].z));
		colors->push_back(osg::Vec4(points[i].r, points[i].g, points[i].b, w));
	}
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

cylinder_func::cylinder_func()
	:axis(),
	radius(0),
	height(0)
{

}

void points_on_line(std::vector<point_3d> &points, std::vector<point_3d> &points_on_line, line_func_3d &line_func, float distance_threshold)
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

void distance_point_to_line(const point_3d &point, const line_func_3d &_line_func_3d, float &points_dis)
{
	point_3d pedal_point;

	pedalpoint_point_to_line(point, _line_func_3d, pedal_point);

	distance_point_to_point(point, pedal_point, points_dis);
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

void distance_point_to_point(const point_3d & point_1, const point_3d & point_2, float & distance)
{
	distance = sqrt(
		(point_1.x - point_2.x)*(point_1.x - point_2.x) +
		(point_1.y - point_2.y)*(point_1.y - point_2.y) +
		(point_1.z - point_2.z)*(point_1.z - point_2.z));
}

void centroid_from_points(std::vector<point_3d> &points, point_3d &centroid_point)
{
	float sum_xyz[3] = { 0,0,0 };

	for (size_t i = 0; i < points.size(); ++i)
	{
		sum_xyz[0] += points[i].x;

		sum_xyz[1] += points[i].y;

		sum_xyz[2] += points[i].z;
	}

	centroid_point.set_xyz(sum_xyz[0] / points.size(), sum_xyz[1] / points.size(), sum_xyz[2] / points.size());
}

void longgest_distance_from_point_to_points(std::vector<point_3d> &points, point_3d &point, float &longgest_distance)
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

void intersection_line_to_sphere(line_func_3d &line_func, point_3d &sphere_center, float &sphere_r, point_3d &res_p1, point_3d &res_p2)
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

	std::vector<point_3d> tmp_p_vec;

	for (size_t i = 0; i < 2; i++)
	{
		for (size_t j = 0; j < 2; j++)
		{
			for (size_t k = 0; k < 2; k++)
			{
				point_3d possible_p(x[i], y[j], z[k]);

				v[0] = possible_p.x - point.x;
				v[1] = possible_p.y - point.y;
				v[2] = possible_p.z - point.z;
				if (is_parallel_vector(line_dir, v))
				{
					// insert without same one 
					bool find_same_one = false;
					for (auto &tmp_p : tmp_p_vec)
						if (possible_p.x == tmp_p.x && possible_p.y == tmp_p.y && possible_p.z == tmp_p.z)
						{
							find_same_one = true;
						}
					if (!find_same_one) tmp_p_vec.push_back(possible_p);
				}
			}
		}
	}

    if (tmp_p_vec.size() == 2)
    {
        result_p1 = tmp_p_vec[0];
        result_p2 = tmp_p_vec[1];
    }
    else
    {
        //std::cout << "[warning] point_along_with_vector_within_dis" << std::endl;
        result_p1 = tmp_p_vec[0];
        result_p2 = tmp_p_vec[1];
    }
}

bool is_parallel_vector(const Eigen::Vector3f & v1, const Eigen::Vector3f & v2)
{
	//std::cout << v1.cross(v2).norm() << std::endl;
	if (v1.cross(v2).norm() < 0.01f)
	{
		return true;
	}
	return false;
}

void max_min_point_3d_vec(std::vector<point_3d> &points, point_3d &min_p, point_3d &max_p)
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

void pedalpoint_point_to_plane(const point_3d &point, const plane_func_3d &plane_func, point_3d &pedalpoint)
{
    float
            t = (point.x * plane_func.a + point.y * plane_func.b + point.z * plane_func.c + plane_func.d)
            / (plane_func.a*plane_func.a + plane_func.b*plane_func.b + plane_func.c*plane_func.c);

    pedalpoint.x = point.x - plane_func.a * t;
    pedalpoint.y = point.y - plane_func.b * t;
    pedalpoint.z = point.z - plane_func.c * t;
}

void make_points_ordered_by_distance(std::vector<point_3d> &points, std::vector<point_3d> &ordered_points)
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
}
