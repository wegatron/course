#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/registration/icp_nl.h>
#include <pcl/io/pcd_io.h>
#include <zsw_vtk_io.h>
#include "transformation_estimate_adv.h"
#include "transformation_estimate_adv.hpp"

int main(int argc, char* argv[])
{
	pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> reg;
	reg.setTransformationEpsilon(1e-6);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance(0.4);
	reg.setMaximumIterations(50);
	pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::TransformationEstimationPtr est(new pcl::registration::TransformationEstimationLM_adv<pcl::PointXYZ, pcl::PointXYZ>);
	reg.setTransformationEstimation(est);
	// Set the point representation
	// 
	// reg.addCorrespondenceRejector(rejector);

	auto conv_crit = reg.getConvergeCriteria();
	// do seting for conv_crit

	pcl::PointCloud<pcl::PointXYZ>::Ptr points_with_normals_src(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr points_with_normals_tgt(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile("/media/wegatron/data/data/pcl_data/tutorials/room_scan2.pcd", *points_with_normals_src);
	pcl::io::loadPCDFile("/media/wegatron/data/data/pcl_data/tutorials/room_scan1.pcd", *points_with_normals_tgt);
    // zsw::point_cloud2vtk_file("/home/wegatron/tmp/target.vtk", points_with_normals_tgt, {});
    // exit(0);

	reg.setInputSource(points_with_normals_src);
	reg.setInputTarget(points_with_normals_tgt);
	//
	// Run the same optimization in a loop and visualize the results
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity();
	pcl::PointCloud<pcl::PointXYZ>::Ptr reg_result = points_with_normals_src;
	reg.setMaximumIterations(2);
	Eigen::AngleAxisf ax(1.0/6 * M_PI, Eigen::Vector3f::UnitZ());
	Eigen::Matrix4f rtmat = Eigen::Matrix4f::Identity();
	rtmat.block<3,3>(0,0) = ax.matrix();
	rtmat(0,3) = 2;
	rtmat(1,3) = 0.3;
	for (int i = 0; i < 30; ++i)
	{
		std::cout << "------------------------::: " << i << std::endl;
		// save cloud for visualization purpose
		points_with_normals_src = reg_result;
		// Estimate
		reg.setInputSource(points_with_normals_src);
		if(i == 0) reg.align(*reg_result, rtmat);
		else reg.align(*reg_result);
		//accumulate transformation between each Iteration
		Ti = reg.getFinalTransformation() * Ti;
		std::cout << Ti << std::endl;
		std::stringstream ss;
		ss<< "/home/wegatron/tmp/reg_" << std::setw(4) << std::setfill('0') << i << ".vtk";
		zsw::point_cloud2vtk_file(ss.str(), reg_result, {});
	}

	return 0;
}
