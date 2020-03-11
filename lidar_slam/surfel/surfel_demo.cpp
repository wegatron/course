#include "surfel.h"
#include "lasser3d_io.h"
#include "zsw_vtk_io.h"
#include <pcl/common/common.h>

int main(int argc, char* argv[])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc = zsw_gz_common::load_laser3d_frame("F:/data/3d-data/off_data_eastcom_floor3/OffData-20180730-velodyne-eastcom-floor3/LASER3D_0_0_2073.88.bin");

	pcl::PointXYZ min_pt, max_pt;
	pcl::getMinMax3D(*pc, min_pt, max_pt);
	std::vector<zsw::surfel> surfel_features = zsw::extract_surfels_basic(pc, min_pt, max_pt, 2, 30);

	const int len = surfel_features.size();
	std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pts(len);
	for(int i=0; i<len; ++i)
	{
		pts[i] = surfel_features[i].des.block<3, 1>(0, 0);
	}
	std::vector<zsw::tensor_mat, Eigen::aligned_allocator<zsw::tensor_mat>> tensor_data(len);
	for(int i=0; i<len; ++i)
	{
		tensor_data[i].block<1, 3>(0, 0) = surfel_features[i].eigen_vector.block<3, 1>(0, 0) * surfel_features[i].eigen_value[0];
		tensor_data[i].block<1, 3>(1, 0) = surfel_features[i].eigen_vector.block<3, 1>(0, 1) * surfel_features[i].eigen_value[1];
		tensor_data[i].block<1, 3>(2, 0) = surfel_features[i].eigen_vector.block<3, 1>(0, 2) * surfel_features[i].eigen_value[2];
	}

	zsw::tensor2vtk_file("f:/tmp.vtk", pts, "surfel", tensor_data);
	
	// std::vector<zsw::tensor_mat, Eigen::aligned_allocator<zsw::tensor_mat>> tensor_data_raw(len);
	// for (int i = 0; i < len; ++i)
	// {
	// 	tensor_data_raw[i] = surfel_features[i].raw;
	// }
	//
	// zsw::tensor2vtk_file("f:/tmp_raw.vtk", pts, "surfel", tensor_data_raw);

	zsw::point_clouds2vtk_file("f:/tmp_ref.vtk", { pc });

	return 0;

}
