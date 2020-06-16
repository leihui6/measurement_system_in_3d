#ifndef INTERFACE_COMMAND
#define INTERFACE_COMMAND

#include <iostream>
#include <boost/thread/thread.hpp>

#include "cloud_viewer.h"

class cloud_viewer;

enum detected_type
{
	DT_EMPTY,
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
	detected_type dt;

	// command_status
	command_status cs;

	//command_status_for_cylinder cs_for_cylinder;

private:
	void run_itself();

	void print_menu();

	// [0-2] for cylinder 
	//	[0] for menu in cylinder step
	//	[1] for alerting
	//	[2] for exit
	// [3] for line quit 
	// [4] for plane quit
	void print_menu_in_steps(int step);

	void clear_picked_points();

	void clear_shapes();

	void print_marked_info();

	cloud_viewer * m_cloud_view_ptr;
};
 
#endif // INTERFACE_COMMAND
