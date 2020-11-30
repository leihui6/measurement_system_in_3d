#include "cloud_pickhandler.h"


PickHandler::PickHandler()
{
	
}

void PickHandler::set_viewer_ptr(cloud_viewer * _cloud_viewer)
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
		// click point
	case osgGA::GUIEventAdapter::DOUBLECLICK:
		pick_status = 1;
		break;
		// remove point
	case osgGA::GUIEventAdapter::PUSH:
		if (ea.getButton() & osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
			pick_status = 2;
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
		if (viewer && m_cloud_viewer->get_target_points())
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

	if (viewer && m_cloud_viewer->get_target_points())
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
					//std::cout << "removed done" << std::endl;
				}
			}
			else
			{
				//std::cout << "No points clicked can be cancelled, please be closer to the point" << std::endl;
			}
		}

		// only test
		// m_picked_points = *m_cloud_viewer->get_target_points();

		// update shapes by user's option
		update_shapes();

		// update: showing on screen in real time
		m_cloud_viewer->update_selected_point_cloud(m_cloud_viewer->m_picked_points, 255, 255, 255, m_cloud_viewer->m_viewer_parameters.picked_point_size);

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

	if (!target_points)
	{
		return false;
	}

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
	if (!this->m_cloud_viewer->m_ic_ptr)
	{
		return;
	}

	if (this->m_cloud_viewer->m_ic_ptr->m_cs == CS_QUIT)
	{
		std::cout << "shapes detection terminated by user\n";

		return;
	}

	if (this->m_cloud_viewer->m_ic_ptr->m_dt == DT_LINE)
	{
		//std::cout << "updating the line model ... " << std::endl;

		process_line();

		//std::cout << "line model updated done " << std::endl;
	}
	else if (this->m_cloud_viewer->m_ic_ptr->m_dt == DT_PLANE)
	{
		//std::cout << "updating the plane model ... " << std::endl;

		plane_func_3d plane_func;

		process_plane(plane_func);

		//std::cout << "plane model updated done " << std::endl;
	}
	else if (this->m_cloud_viewer->m_ic_ptr->m_dt == DT_CYLINDER)
	{
		//std::cout << "updating the cylinder model ... " << std::endl;

		process_cylinder();

		//std::cout << "cylinder model updated done " << std::endl;
	}
	else if (this->m_cloud_viewer->m_ic_ptr->m_dt == DT_POINT)
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
	if (m_cloud_viewer->m_picked_points.size() < 2) return;

	// calculate the fitting function
	m_cloud_viewer->m_line_points = m_cloud_viewer->m_picked_points;
	line_func_3d line_func;
	m_cloud_viewer->m_cf.fitting_line_3d_linear_least_squares(m_cloud_viewer->m_picked_points, line_func);

	// search points with respect to the fitted plane function
	//point_3d sphere_center;
	//float sphere_r;
	//centroid_from_points(m_cloud_viewer->m_picked_points, sphere_center);
	//mean_distance_from_point_to_points(m_cloud_viewer->m_picked_points, sphere_center, sphere_r);
	//longgest_distance_from_point_to_points(m_cloud_viewer->m_picked_points, sphere_center, sphere_r);
	// sphere_r = sphere_r + 0.2*sphere_r;
	// calculate the segment points for visualization
	std::vector<point_3d> real_line_segment(2);
	segment_point_from_points(real_line_segment[0], real_line_segment[1], m_cloud_viewer->m_picked_points, line_func);
	//intersection_line_to_sphere(line_func, sphere_center, sphere_r, real_line_segment[0], real_line_segment[1]);

	// pick up more points based on picked points
	if (m_cloud_viewer->m_viewer_parameters.is_auto_pick)
	{
		//std::vector<point_3d> points_in_sphere, _points_on_line;
		//m_cloud_viewer->m_kdtree.search_neighbors_radius(sphere_r, sphere_center, points_in_sphere);
		//points_on_line(points_in_sphere, _points_on_line, line_func, m_cloud_viewer->m_viewer_parameters.auto_pick_line_threshold);
		//add_points_to_picked_vector(_points_on_line);
	}
	m_cloud_viewer->update_line(real_line_segment, 0, 255, 0, m_cloud_viewer->m_viewer_parameters.fitting_line_width);
}

void PickHandler::process_plane(plane_func_3d & plane_func)
{
	if (m_cloud_viewer->m_picked_points.size() < 3) return;

	m_cloud_viewer->m_plane_points = m_cloud_viewer->m_picked_points;

	m_cloud_viewer->m_cf.fitting_plane_3d_linear_least_squares(m_cloud_viewer->m_picked_points, plane_func);

	point_3d min_p, max_p;

	max_min_point_3d_vec(m_cloud_viewer->m_picked_points, min_p, max_p);

	pedalpoint_point_to_plane(min_p, plane_func, min_p);

	pedalpoint_point_to_plane(max_p, plane_func, max_p);

	// using four points to draw a biggest rectangle
	Eigen::Vector3f diagonal, line_direction;
	diagonal = Eigen::Vector3f(max_p.x - min_p.x, max_p.y - min_p.y, max_p.z - min_p.z);
	line_direction = diagonal.cross(Eigen::Vector3f(plane_func.a, plane_func.b, plane_func.c));

	point_3d mid_point, corner_p1, corner_p2;
	mid_point = point_3d((min_p.x + max_p.x) / 2, (min_p.y + max_p.y) / 2, (min_p.z + max_p.z) / 2);
	float half_dis = diagonal.norm() / 2;

	point_along_with_vector_within_dis(mid_point, line_direction, corner_p1, corner_p2, half_dis);

	// auto update plane with more points
	if (m_cloud_viewer->m_viewer_parameters.is_auto_pick)
	{
		//std::vector<point_3d> points_in_sphere, _points_on_plane;
		//m_cloud_viewer->m_kdtree.search_neighbors_radius(half_dis, mid_point, points_in_sphere);
		//points_on_plane(points_in_sphere, _points_on_plane, plane_func, m_cloud_viewer->m_viewer_parameters.auto_pick_plane_threshold);
		//add_points_to_picked_vector(_points_on_plane);
	}

	std::vector<point_3d> biggest_rectangle{ min_p,max_p,corner_p1,corner_p2 }, drawable_points_ordered;
	make_points_ordered_by_distance(biggest_rectangle, drawable_points_ordered);
	m_cloud_viewer->update_plane(drawable_points_ordered, 0, 255, 0);
}

void PickHandler::process_cylinder()
{
	if (m_cloud_viewer->m_picked_points.size() < 6)
	{
		return;
	}

	//if (m_cloud_viewer->m_ic_ptr->m_cs == CS_SELECTING_POINTS)
	//{
		//std::cout << "please marking points represent the bottom plane of cylinder.\n";

		//m_cloud_viewer->m_picked_points = *m_cloud_viewer->get_target_points();

	m_cloud_viewer->m_cf.fitting_cylinder_linear_least_squares(m_cloud_viewer->m_picked_points, m_cylinder_func);

	//std::cout << m_cylinder_func.height << " radius=" << m_cylinder_func.radius << std::endl;

	//// 0) skip the last point which mean the height point
	//point_3d height_p;
	//plane_func_3d cylinder_bottom_plane_func;
	//std::vector<point_3d> bottom_circle_points;

	//bottom_circle_points = m_cloud_viewer->m_picked_points;
	//height_p = bottom_circle_points.back();
	//bottom_circle_points.pop_back();

	//// 1) get one center point on bottom plane and R
	//centroid_from_points(bottom_circle_points, m_centriod_point_on_bottom);
	//mean_distance_from_point_to_points(bottom_circle_points, m_centriod_point_on_bottom, m_cylinder_func.r);
	//m_cloud_viewer->m_cf.fitting_plane_3d_linear_least_squares(bottom_circle_points, cylinder_bottom_plane_func);
	//
	//// find other more points on this bottom plane
	//bottom_circle_points.clear();
	//points_on_plane_circle(*m_cloud_viewer->get_target_points(), bottom_circle_points,
	//	cylinder_bottom_plane_func, m_centriod_point_on_bottom, m_cylinder_func.r, 0.8, 1.0);
	//m_cloud_viewer->update_testing_point_cloud(bottom_circle_points, 255, 255, 0, 4.0f);

	//// 2) calculate it again to update cylinder info, that is get new one center point on bottom plane and R
	//centroid_from_points(bottom_circle_points, m_centriod_point_on_bottom);
	//mean_distance_from_point_to_points(bottom_circle_points, m_centriod_point_on_bottom, m_cylinder_func.r);
	//m_cloud_viewer->m_cf.fitting_plane_3d_linear_least_squares(bottom_circle_points, cylinder_bottom_plane_func);

	//bottom_circle_points.clear();
	//points_on_plane_circle(*m_cloud_viewer->get_target_points(), bottom_circle_points,
	//	cylinder_bottom_plane_func, m_centriod_point_on_bottom, m_cylinder_func.r, 0.8, 1.0);

	//// 3) create current cylinder function
	//m_cylinder_func.axis.set_nml(cylinder_bottom_plane_func.a, cylinder_bottom_plane_func.b, cylinder_bottom_plane_func.c);
	//m_cylinder_func.axis.set_xyz(m_centriod_point_on_bottom.x, m_centriod_point_on_bottom.y, m_centriod_point_on_bottom.z);
	//pedalpoint_point_to_line(height_p, m_cylinder_func.axis, m_cylinder_func.top_plane_point);

	//// 4) However, we cannot use this cylinder function to show a cylinder on screen
	Eigen::Vector3f z_axis(0, 0, 1);

	Eigen::Vector3f  rotated_axis = z_axis.cross(m_cylinder_func.axis.direction);

	float rotated_angle = 0.0;
	angle_between_two_vector_3d(z_axis, m_cylinder_func.axis.direction, rotated_angle);

	//std::cout << "rotated_angle=" << rotated_angle << std::endl;

	m_cloud_viewer->m_cylinder_points = m_cloud_viewer->m_picked_points;

	m_cloud_viewer->update_cylinder(
		m_cylinder_func,
		rotated_axis,
		rotated_angle,
		0, 255, 0, 0.5);

	/*
	std::vector<point_3d> cpb{ m_centriod_point_on_bottom };

	// black color to show center point on bottom
	m_cloud_viewer->update_cylinder_centriod_point_on_bottom(cpb, 0, 0, 0, 15);

	// set the cylinder function
	m_cylinder_func.m_line_func.set_nml(m_cylinder_plane_func.a, m_cylinder_plane_func.b, m_cylinder_plane_func.c);

	m_cylinder_func.m_line_func.set_xyz(m_centriod_point_on_bottom.x, m_centriod_point_on_bottom.y, m_centriod_point_on_bottom.z);

	mean_distance_from_point_to_points(m_cloud_viewer->m_picked_points, m_centriod_point_on_bottom, m_cylinder_func.r);

	points_on_cylinder(*m_cloud_viewer->get_target_points(), m_cloud_viewer->m_cylinder_points, m_cylinder_func, m_cylinder_func.r, 0.3);

	m_cloud_viewer->update_cylinder(m_cloud_viewer->m_cylinder_points, 255, 255, 0, 4);
	*/

	//}
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
			//std::cout << "point existed" << std::endl;
			return false;
		}
	}

	m_cloud_viewer->m_picked_points.push_back(p);

	return true;
}

void PickHandler::add_points_to_picked_vector(std::vector<point_3d>& vec)
{
	for (size_t i = 0; i < vec.size(); ++i)
	{
		add_point_to_picked_vector(vec[i]);
	}
}

bool PickHandler::remove_point_from_picked_vector(const point_3d & p)
{
	for (std::vector<point_3d>::iterator it = m_cloud_viewer->m_picked_points.begin();
		it != m_cloud_viewer->m_picked_points.end(); ++it)
	{
		if (it->x == p.x &&it->y == p.y &&it->z == p.z)
		{
			//std::cout << "point unclicked from picked set" << std::endl;

			it = m_cloud_viewer->m_picked_points.erase(it);

			return true;
		}
	}

	std::cout << "point doesn't exist in picked set" << std::endl;

	return false;
}
