
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <lasser3d_io.h>
#include <zsw_vtk_io.h>

int main(int argc, char* argv[])
{
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> laser_pcs;
	std::vector<Eigen::Affine3f> transforms;
	zsw_gz_common::load_graph_nodes("F:/data/3d-data/chaosuan/Node3DDir-20190517T102957/graph_20190517T132930.txt", laser_pcs, transforms);
	const int num = laser_pcs.size();
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> final_laser_pcs(num, nullptr);
#pragma omp parallel for
	for(int i=0; i< num; ++i)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr tr_lgp(new pcl::PointCloud<pcl::PointXYZ>);
		transformPointCloud(*laser_pcs[i], *tr_lgp, transforms[i]);
		final_laser_pcs[i] = tr_lgp;
	}
	zsw::point_clouds2vtk_file("analysis.vtk", final_laser_pcs);
	return 0;
}
