#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "lasser3d_io.h"
#include "zsw_vtk_io.h"

#define DATA_DIR "/media/wegatron/data/data/data/tutorials/"

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_0 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
  //// Fill in the cloud data
  //pcl::PCDReader reader;
  //// Replace the path below with the path where you saved your file
  //std::string data_file = std::string(DATA_DIR)+"table_scene_lms400.pcd";
  //reader.read<pcl::PointXYZ>(data_file, *cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = zsw_gz_common::load_pts_binary("F:/data/3d-data/chaosuan/zsw_latest/graph_cloud_20190522T132857.bin");
  zsw::point_cloud2vtk_file("ori.vtk", cloud, {});

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-0.7, 2.0);
  pass.filter(*cloud_0);
  zsw::point_cloud2vtk_file("removed_platfond.vtk", cloud_0, {});

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud_0 << std::endl;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_0);
  sor.setMeanK (50);
  sor.setStddevMulThresh (4);
  sor.filter (*cloud_1);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_1 << std::endl;

  zsw::point_cloud2vtk_file("inlier.vtk", cloud_1, {});
  zsw_gz_common::write_pts_binary("final_pc.bin", *cloud_1);


  sor.setNegative(true);
  sor.filter(*cloud_1);
  zsw::point_cloud2vtk_file("outlier.vtk", cloud_1, {});

  return 0;
}
