#ifndef CLOUD_GEOMETRY_H
#define CLOUD_GEOMETRY_H

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <cfloat>

#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Point>

#include <Eigen/Dense>

struct point_3d
{
    point_3d(const point_3d & p);

    point_3d();

    point_3d(float x, float y, float z);

    void set_xyz(float x, float y, float z);

    void set_nxyz(float nx, float ny, float nz);

    void set_rgb(float r, float g, float b);

    Eigen::Vector3f get_vector3f();

    friend std::ostream & operator << (std::ostream & os, const point_3d & p);

    point_3d & operator = (const point_3d & p);
    point_3d operator + (const point_3d & p);
    point_3d operator / (const float num);

    float x, y, z;

    float nx, ny, nz;

    float r, g, b;
};

struct line_func_3d
{
    line_func_3d();

    void set_xyz(float x, float y, float z);

    void set_nml(float n, float m, float l);

    point_3d get_origin_point_3d();

    point_3d get_direction_point_3d();

    //float x, y, z;
    Eigen::Vector3f origin;

    //float n, m, l;
    Eigen::Vector3f direction;
};


struct plane_func_3d
{
    plane_func_3d();

    void set_abcd(float _a, float _b, float _c, float _d);

    void set_abcd(float _a, float _b, float _c, point_3d & p);

    template <typename T>
    T direction()
    {
        return T(this->a, this->b, this->c);
    }

    float a, b, c, d;
};

struct cylinder_func
{
    cylinder_func();

    line_func_3d axis;

    float radius;

    float height;
};

void points_to_osg_structure(std::vector<point_3d>& points, osg::ref_ptr<osg::Vec3Array> coords, osg::ref_ptr<osg::Vec4Array> colors, float w);
void points_to_geometry_node(std::vector<point_3d>& points, osg::ref_ptr<osg::Geometry> geometry, float w);

void points_on_line(std::vector<point_3d>& points, std::vector<point_3d>& points_on_line, line_func_3d & line_func, float distance_threshold);

void distance_point_to_line(const point_3d& points, const line_func_3d & _line_func_3d, float & points_dis);
void distance_point_to_point(const point_3d & point_1, const point_3d & point_2, float & distance);
void longgest_distance_from_point_to_points(std::vector<point_3d>& points, point_3d & point, float & longgest_distance);

void pedalpoint_point_to_line(const point_3d & point, const line_func_3d & _line_func_3d, point_3d & pedalpoint);
void pedalpoint_point_to_plane(const point_3d & point, const plane_func_3d & plane_func, point_3d & pedalpoint);
void centroid_from_points(std::vector<point_3d>& points, point_3d & centroid_point);
void intersection_line_to_sphere(line_func_3d & line_func, point_3d & sphere_center, float & sphere_r, point_3d & res_p1, point_3d & res_p2);
void point_along_with_vector_within_dis(point_3d & point, Eigen::Vector3f & line_dir, point_3d & result_p1, point_3d & result_p2, float distance);

void max_min_point_3d_vec(std::vector<point_3d> & points, point_3d & min_p, point_3d & max_p);

bool is_parallel_vector(const Eigen::Vector3f & v1, const Eigen::Vector3f & v2);

void make_points_ordered_by_distance(std::vector<point_3d>& points, std::vector<point_3d>& ordered_points);

#endif // CLOUD_GEOMETRY_H
