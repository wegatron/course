
#include <lasser3d_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <zsw_vtk_io.h>
#include <fstream>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>

int main(int argc, char* argv[])
{
	std::ifstream gifs("F:/data/tmp/beijin_IDC_res/lasser_res_pose.txt", std::ifstream::binary);
	std::string prefix = "F:/data/tmp/beijin_IDC_res/lasser_";
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> points_clouds;
	for (int i = 11; i < 13; ++i)
	{
		gifs.seekg(i * 16 * sizeof(double), gifs.beg);
		Eigen::Matrix4f rt;
		gifs.read(reinterpret_cast<char*>(rt.data()), sizeof(double) * 16);
		std::string lasser_file = prefix + std::to_string(i) + ".bin";
		auto raw_pc = zsw_gz_common::load_laser3d_frame(lasser_file);
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);

		for(auto &pt : raw_pc->points)
		{
			Eigen::Vector3f p(pt.x, pt.y, pt.z);
			if(p.squaredNorm() <= 10000)
			{
				pc->push_back(pt);
			}
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*pc, *pc_out, rt);
		points_clouds.push_back(pc_out);
	}

	zsw::point_clouds2vtk_file("f:/data/tmp/tmp_res.vtk", points_clouds);
	return 0;
}
