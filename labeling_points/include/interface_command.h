#ifndef INTERFACE_COMMAND
#define INTERFACE_COMMAND

#include <iostream>
#include <boost/thread/thread.hpp>

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

class interface_command
{
public:
	interface_command();

	~interface_command();

	void run();

	// detected_type
	detected_type dt;

	// command_status
	command_status cs;

private:
	void run_itself();

	void print_menu();

};
 
#endif // INTERFACE_COMMAND
