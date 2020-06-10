#include "interface_command.h"



interface_command::interface_command()
	:cs(CS_NORMAL),
	dt(DT_EMPTY)
{
}


interface_command::~interface_command()
{
}

void interface_command::run()
{
	boost::thread run_thread(boost::bind(&interface_command::run_itself, this));

	run_thread.detach();
}

void interface_command::run_itself()
{
	char i_c;

	print_menu();

	while (true)
	{
		std::cin >> i_c;

		if (i_c == 'q')
		{
			// quit selecting model
			if (cs == CS_SELECTING_POINTS)
			{
				std::cout << "Exit manually selecting step" << std::endl;

				cs = CS_NORMAL;

				print_menu();

				continue;
			}
			// quit command interface
			else
			{
				std::cout << "exit manually console" << std::endl;

				cs = CS_QUIT;

				dt = DT_EMPTY;

				break;
			}
			
		}
		else if (i_c == 'a')
		{
			std::cout << "you choosed \"LINE\" type, please select the points in the scene" << std::endl;

			cs = CS_SELECTING_POINTS;

			dt = DT_LINE;

			print_menu();
		}
		else if (i_c == 'b')
		{
			std::cout << "you choosed \"PLANE\" type, please select the points in the scene" << std::endl;

			cs = CS_SELECTING_POINTS;

			dt = DT_PLANE;

			print_menu();
		}
		else if (i_c == 'c')
		{
			std::cout << "you choosed  \"CYLINDER\" type, please select the points in the scene" << std::endl;

			cs = CS_SELECTING_POINTS;

			dt = DT_CYLINDER;

			print_menu();
		}
		else
		{
			print_menu();
		}
	}
}

void interface_command::print_menu()
{
	if (this->cs == CS_SELECTING_POINTS)
	{
		std::cout
			<< "please enter \"q\" to quit current step:";
	}
	else
	{
		std::cout
			<< "please choice the shape that you want to detect, \"q\" to quit" << "\n"
			<< "a) 3d line" << "\n"
			<< "b) 3d plane" << "\n"
			<< "c) 3d cylinder" << "\n"
			<< "choose an option:";
	}
}
