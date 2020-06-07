#ifndef INTERFACE_COMMAND
#define INTERFACE_COMMAND

#include <iostream>

enum detected_type
{
	EMPTY,
	LINE,
	PLANE,
	CYLINDER
};

class interface_command
{
public:
	interface_command();

	~interface_command();

	void run();

	detected_type dt;
};
 
#endif // INTERFACE_COMMAND
