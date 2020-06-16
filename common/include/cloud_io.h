#ifndef CLOUD_IO_H
#define CLOUD_IO_H

#include <vector>
#include <fstream>
#include <sstream>

#include "cloud_geometry.h"
#include "cloud_processing.h"

bool load_point_cloud_txt(const std::string & filename, std::vector<point_3d> & points, bool fill_normals = false);

bool load_point_cloud_vtk(const std::string & filename, std::vector<point_3d> & points);

#endif // CLOUD_IO_H



