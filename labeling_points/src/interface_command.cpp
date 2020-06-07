#include "interface_command.h"



interface_command::interface_command()
{
}


interface_command::~interface_command()
{
}

void interface_command::run()
{
	char i_c;

	while (true)
	{
		std::cin >> i_c;

		if (i_c == 'q')
		{
			std::cout << "Exit manually" << std::endl;

			break;
		}
		else
		{

		}

	}
}
