#ifndef COMMON_USE
#define COMMON_USE

#include <Eigen/Dense>

#include <filesystem>

#include "cloud_viewer.h"

#include "cloud_io.h"

// write one matrix(4x4) to file
extern void save_matrix(Eigen::Matrix4f & matrix, const std::string & file_name);

// read one matrix(4x4) from file
extern Eigen::Matrix4f read_matrix(const std::string & file_name);

// cut '/' and postfix context of file name string 
extern std::string file_name_without_postfix(std::string & file_name);

// check if the file exists
extern bool is_exist(const std::string & file_name);

// load more file containing matrix into a vector consists of Eigen::Matrix4f
extern void load_file_to_vec(std::string & folder, std::vector<Eigen::Matrix4f> & m_vec);

// display point cloud in osg repeatly with mroe matrices
extern void display_point_cloud_from_transformation_vec(cloud_viewer & cv, std::vector<point_3d> & reading_point_cloud, std::vector<Eigen::Matrix4f> &transformation_vec);

// read points from file that containing many points and spilted by '#'
extern void read_points(std::map<std::string, std::vector<point_3d>> & points_map, const std::string & file_name);

#endif // COMMON_USE
