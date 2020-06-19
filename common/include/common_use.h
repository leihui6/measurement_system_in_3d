#ifndef COMMON_USE
#define COMMON_USE

#include <Eigen/Dense>

#include <filesystem>

#include "cloud_viewer.h"

#include "cloud_io.h"

//#define CORASE_REGISTRATION
//#define FINE_REGISTRATION
//#define LABELING_POINTS

extern void save_matrix(Eigen::Matrix4f & matrix, const std::string & file_name);

extern Eigen::Matrix4f read_matrix(const std::string & file_name);

extern std::string file_name_without_postfix(std::string & file_name);

extern void load_file_to_display(std::string & folder, cloud_viewer * cv, float r = 0, float g = 255, float b = 0, float point_size = 4.0, size_t interval_time = 1000);

#endif // COMMON_USE
