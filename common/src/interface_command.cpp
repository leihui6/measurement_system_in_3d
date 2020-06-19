#include "interface_command.h"

interface_command::interface_command(cloud_viewer * cloud_view_ptr)
	:cs(CS_NORMAL),
	dt(DT_EMPTY)
{
	m_cloud_view_ptr = cloud_view_ptr;

	m_cloud_view_ptr->set_the_interface_command(this);
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

	while (true)
	{
		// menu
		print_menu();

		std::cin >> i_c;

		if (i_c == 'q')
		{
			std::cout << "exited labeling process manually" << std::endl;

			cs = CS_QUIT;

			dt = DT_EMPTY;

			break;
		}
		// points
		else if (i_c == 'a')
		{
			clear_picked_points();

			clear_shapes();

			std::cout << "you choosed \"POINT\" type, please select the points in the scene" << std::endl;

			cs = CS_SELECTING_POINTS;

			dt = DT_POINT;

			while (true)
			{
				print_menu();

				std::cin >> i_c;

				if (i_c == 'q')
				{
					cs = CS_NORMAL;

					dt = DT_EMPTY;

					m_cloud_view_ptr->save_points_to_vec(m_cloud_view_ptr->m_points, m_cloud_view_ptr->m_points_vec);

					print_menu_in_steps(5);

					break;
				}
			}
		}
		// line
		else if (i_c == 'b')
		{
			clear_picked_points();

			clear_shapes();

			std::cout << "you choosed \"LINE\" type, please select the points in the scene" << std::endl;

			cs = CS_SELECTING_POINTS;

			dt = DT_LINE;

			while (true)
			{
				print_menu();

				std::cin >> i_c;

				if (i_c == 'q')
				{
					cs = CS_NORMAL;

					dt = DT_EMPTY;

					m_cloud_view_ptr->save_points_to_vec(m_cloud_view_ptr->m_line_points, m_cloud_view_ptr->m_line_points_vec);

					print_menu_in_steps(4);

					break;
				}
			}
		}
		// plane
		else if (i_c == 'c')
		{
			clear_picked_points();

			clear_shapes();

			std::cout << "you choosed \"PLANE\" type, please select the points in the scene" << std::endl;

			cs = CS_SELECTING_POINTS;

			dt = DT_PLANE;

			while (true)
			{
				print_menu();

				std::cin >> i_c;

				if (i_c == 'q')
				{
					cs = CS_NORMAL;

					dt = DT_EMPTY;

					m_cloud_view_ptr->save_points_to_vec(m_cloud_view_ptr->m_plane_points, m_cloud_view_ptr->m_plane_points_vec);

					print_menu_in_steps(3);

					break;
				}
			}
		}
		// cylinder
		else if (i_c == 'd')
		{
			clear_picked_points();

			clear_shapes();

			std::cout << "you choosed  \"CYLINDER\" type, please select the points in the scene" << std::endl;

			cs = CS_SELECTING_POINTS;

			dt = DT_CYLINDER;

			print_menu_in_steps(0);

			while (true)
			{
				print_menu_in_steps(1);

				std::cin >> i_c;

				if (i_c == 'q')
				{
					cs = CS_NORMAL;

					dt = DT_EMPTY;

					m_cloud_view_ptr->save_points_to_vec(m_cloud_view_ptr->m_cylinder_points, m_cloud_view_ptr->m_cylinder_points_vec);

					print_menu_in_steps(2);

					break;
				}
			}
		}// end for c
		else if (i_c == 's')
		{
			m_cloud_view_ptr->print_marked_info();
		}
		else if (i_c == 'e')
		{
			m_cloud_view_ptr->export_points();

			std::cout << "exported done!\n";
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
			<< "please choice the shape that you want to detect, press \"q\" to quit" << "\n"
			<< "a) 3d point" << "\n"
			<< "b) 3d line" << "\n"
			<< "c) 3d plane" << "\n"
			<< "d) 3d cylinder" << "\n"
			<< "s) show all shapes marked" << "\n"
			<< "e) export all marked points" << "\n"
			<< "choose an option:";
	}
}

void interface_command::print_menu_in_steps(int step)
{
	// menu
	if (step == 0)
	{
		std::cout
			<< "note that before marking cylinder, please mark the bottom plane of cylinder, i.e., picking points representing a plane.\n"
			<< "points building a cylinder will be updated in real time, please check if it is a good representation\n"
			<< "(white) points mean picked points\n"
			<< "(green)plane means the bottom of cylinder\n"
			<< "(black)point means the center point on bottom of cylinder\n"
			<< "(blue) points on cylinder, please check it \n";
	}
	else if (step == 1)
	{
		std::cout
			<< "you can press 'q' to quit current step if you are done!. \n";
	}
	//quit
	else if (step == 2)
	{
		std::cout
			<< "quit marking cylinder step.\n";
	}
	else if (step == 3)
	{
		std::cout
			<< "quit marking the lines step.\n";
	}
	else if (step == 4)
	{
		std::cout
			<< "quit marking the planes step.\n";
	}
	else if (step == 5)
	{
		std::cout
			<< "quit marking the points step.\n";
	}
}

void interface_command::clear_picked_points()
{
	m_cloud_view_ptr->clear_picked_points();
}

void interface_command::clear_shapes()
{
	m_cloud_view_ptr->clear_shapes();
}

void interface_command::print_marked_info()
{
	m_cloud_view_ptr->print_marked_info();
}
