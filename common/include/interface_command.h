#ifndef INTERFACE_COMMAND
#define INTERFACE_COMMAND

#include <iostream>
#include <boost/thread/thread.hpp>

#include "cloud_viewer.h"

class cloud_viewer;

enum detected_type
{
	DT_EMPTY,
	DT_POINT,
	DT_LINE,
	DT_PLANE,
	DT_CYLINDER
};

enum command_status
{
	CS_NORMAL,
	CS_SELECTING_POINTS,
	CS_QUIT,
};

//enum command_status_for_cylinder
//{
//	CS_SELECTING_CYLINDER_PLANE,
//	CS_SELECTING_CYLINDER_DONE,
//};

class interface_command
{
public:
	interface_command(cloud_viewer * cloud_view_ptr);

	~interface_command();

	void run();

	// detected_type
	detected_type m_dt;

	// command_status
	command_status m_cs;

	std::vector<size_t> marked_points_count;

	//command_status_for_cylinder cs_for_cylinder;

private:
	void run_itself();

	void print_menu();

	// [0] means beginning [1] means ending
	void print_menu_in_steps(detected_type dt, int flag);

	void clear_picked_points();

	void clear_shapes();

	void print_marked_info();

	cloud_viewer * m_cloud_view_ptr;

	void input_marked_name(std::string & marked_name, const std::string  & default_name);

	void process_specific_type(detected_type _dt);

};
 
#endif // INTERFACE_COMMAND
