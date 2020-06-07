#ifndef CLOUD_IO_H
#define CLOUD_IO_H

#include <vector>
#include <fstream>
#include <sstream>

#include "cloud_geometry.h"

bool load_point_cloud_txt(const std::string & filename, std::vector<point_3d> & points);

#endif // CLOUD_IO_H



