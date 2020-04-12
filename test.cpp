#include "cloud_io.h"
#include "osg_viewer.h"

int main()
{
	std::vector<point_3d> point_3d_vec;

	load_point_cloud_txt("model.asc", point_3d_vec);

	osg_viewer m_osg_viewer;

	m_osg_viewer.add_point_cloud(point_3d_vec);

	return 0;
}