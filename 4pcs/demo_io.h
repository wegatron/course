#ifndef VTK_IO_H
#define VTK_IO_H

#include <boost/regex.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "PointPairFilter.h"

bool read_point_cloud(const std::string& file_path,
                      std::vector<gr::Point3D>& pts);


bool load_laser3d_frame(const std::string& laser3d_file, std::vector<gr::Point3D>& pts);


bool write_vtk(const std::string& file_path, const std::vector<gr::Point3D>& pts);

bool write_vtk2(const std::string& file_path, const std::vector<gr::Point3D>& pts_a,
                const std::vector<gr::Point3D>& pts_b);

bool rt2vtk(const std::string& file_path, const std::vector<Eigen::Matrix<float, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<float, 6, 1>>> &rts);


bool load_local_map(const std::string &graph_file, const Eigen::Vector3f &pos, std::vector<gr::Point3D> &pts);
#endif // VTK_IO_H
