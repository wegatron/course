#include "ground_seg.h"
#include <fstream>
#include <zsw_vtk_io.h>
#include <pcl/common/transforms.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr load_kitti_lasser_bin(const std::string &file)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pts(new pcl::PointCloud<pcl::PointXYZ>);
  int32_t pt_num = 250000;
  static std::vector<float> buffer(pt_num);
  std::ifstream ifs(file, std::ifstream::binary);
  if(!ifs) std::cerr << "file open error" << std::endl;
  ifs.read(reinterpret_cast<char*>(buffer.data()), sizeof(float)*pt_num);
  std::cout << ifs.tellg() << std::endl;
  pt_num = ifs.tellg()/(sizeof(float)*4);
  std::cout << "pt num = " << pt_num << std::endl;
  auto data_ptr = buffer.data();
  pts->resize(pt_num);
  for(int i=0; i<pt_num; ++i) {
    pts->points[i].x = *data_ptr; ++data_ptr;
    pts->points[i].y = *data_ptr; ++data_ptr;
    pts->points[i].z = *data_ptr; data_ptr+=2;
  }
  return pts;
}

int main(int argc, char * argv[])
{
  auto pts_ptr = load_kitti_lasser_bin(argv[1]);
  float height = 1.73; // kitti
  for(auto &pt : pts_ptr->points)
      pt.z += 1.73;

  zsw::point_cloud2vtk_file("/home/wegatron/tmp/tmp.vtk", pts_ptr, {});
  ground_seg_params params;
  gaussian_process_ground_seg gseg("/media/wegatron/data/workspace/zsw_course/point_cloud_course/HW4/config.yaml");
  auto labels = gseg.segment(pts_ptr);

  //output
  return 0;
}
