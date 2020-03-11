#include "remove_small_island.h"

#include <pcl/segmentation/extract_clusters.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkFloatArray.h>
#include <vtkIntArray.h>
#include <vtkPointData.h>
#include <vtkPolyDataWriter.h>
#include <pcl/features/normal_3d.h>

#include "demo_io.h"
#include <lasser3d_io.h>
#include <zsw_vtk_io.h>

void euclidean_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr pcs, const std::string &output_vtk)
{
	
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.4);
	ec.setMinClusterSize(200);
	ec.setMaxClusterSize(25000);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(pcs);
	ec.setSearchMethod(tree);
	ec.setInputCloud(pcs);
	std::vector<pcl::PointIndices> cluster_indices;
	ec.extract(cluster_indices);

	// output result
	vtkSmartPointer<vtkPoints> points_data = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPolyData> vtk_points_cloud = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkIntArray> index = vtkSmartPointer<vtkIntArray>::New();
	vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();

	

	int cur_ind = 0;
	for (const auto &indices : cluster_indices)
	{
		for (const auto &ind : indices.indices)
		{
			auto id = points_data->InsertNextPoint(pcs->points[ind].x, pcs->points[ind].y, pcs->points[ind].z);
			vertices->InsertNextCell(1);
			vertices->InsertCellPoint(id);
			index->InsertNextTuple1(cur_ind);
		}
		++cur_ind;
	}

	vtk_points_cloud->SetPoints(points_data);
	vtk_points_cloud->SetVerts(vertices);
	vtk_points_cloud->GetPointData()->SetScalars(index);
	// write out
	static vtkSmartPointer<vtkPolyDataWriter> poly_data_writer = vtkSmartPointer<vtkPolyDataWriter>::New();
	poly_data_writer->SetInputData(vtk_points_cloud);
	poly_data_writer->SetFileName(output_vtk.c_str());
	poly_data_writer->SetFileTypeToBinary();
	poly_data_writer->Write();
}


void remove_small_island(std::vector<gr::Point3D> &points, const float radius, const int num_pts, const std::string &output_vtk)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcs(new pcl::PointCloud<pcl::PointXYZ>);

	for (auto pt : points)
	{
		pcs->push_back(pcl::PointXYZ(pt.x(), pt.y(), pt.z()));
	}

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(pcs);

	// gen normals
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(pcs);
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.3);
	ne.setViewPoint(0, 0, 0);
	ne.compute(*cloud_normals);
	const int pc_size = points.size();
	for (int i = 0; i < pc_size; ++i)
	{
		points[i].normal_ref() = gr::Point3D::VectorType(cloud_normals->points[i].normal_x, cloud_normals->points[i].normal_y, cloud_normals->points[i].normal_z);
	}

	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(radius);
	ec.setMinClusterSize(num_pts);
	ec.setSearchMethod(tree);
	ec.setInputCloud(pcs);
	std::vector<pcl::PointIndices> cluster_indices;
	ec.extract(cluster_indices);
	
	std::vector<gr::Point3D> result_pcs;
	vtkSmartPointer<vtkFloatArray> points_normal = vtkSmartPointer<vtkFloatArray>::New();
	points_normal->SetNumberOfComponents(3);
	for (const auto &indices : cluster_indices)
	{
		for (const auto &ind : indices.indices)
		{
			result_pcs.push_back(points[ind]);
			points_normal->InsertNextTuple3(points[ind].normal_ref().x(), points[ind].normal_ref().y(), points[ind].normal_ref().z());
		}
	}

	points.swap(result_pcs);

	vtkSmartPointer<vtkPoints> points_data = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPolyData> vtk_points_cloud = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
	
	for (const auto &pt : points)
	{
		auto id = points_data->InsertNextPoint(pt.x(), pt.y(), pt.z());
		vertices->InsertNextCell(1);
		vertices->InsertCellPoint(id);
	}

	vtk_points_cloud->SetPoints(points_data);
	vtk_points_cloud->SetVerts(vertices);
	vtk_points_cloud->GetPointData()->SetNormals(points_normal);

	static vtkSmartPointer<vtkPolyDataWriter> poly_data_writer = vtkSmartPointer<vtkPolyDataWriter>::New();
	poly_data_writer->SetInputData(vtk_points_cloud);
	poly_data_writer->SetFileName(output_vtk.c_str());
	poly_data_writer->SetFileTypeToBinary();
	poly_data_writer->Write();
}

void output_res(std::vector<gr::Point3D> &set1, std::vector<gr::Point3D> &set2)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcs(new pcl::PointCloud<pcl::PointXYZ>);

	for (auto pt : set1)
	{
		pcs->push_back(pcl::PointXYZ(pt.x(), pt.y(), pt.z()));
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcs2(new pcl::PointCloud<pcl::PointXYZ>);

	for (auto pt : set2)
	{
		pcs2->push_back(pcl::PointXYZ(pt.x(), pt.y(), pt.z()));
	}

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(pcs);

	std::vector<float> hosdorff_dis;
	std::vector<int> indices;
	std::vector<float> sqr_dis;
	for (auto pt : set2)
	{
		tree->nearestKSearch(pcl::PointXYZ(pt.x(), pt.y(), pt.z()), 1, indices, sqr_dis);
		hosdorff_dis.push_back(sqrt(sqr_dis[0]));
	}

	zsw::point_cloud2vtk_file("F:/data/tmp/4pcs_res/res_dis.vtk", pcs2, { std::pair<std::string, std::vector<float>>("dis", hosdorff_dis) });
}

//void image_based_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr pcs, const std::string &output_vtk)
//{
//	fast_range_seg<pcl::PointXYZ> fc;
//	fc.setInputCloud(pcs);
//	std::vector<pcl::PointIndices> cluster_indices;
//	fc.extract(cluster_indices);
//}

//int main(int argc, char* argv[])
//{
//	//pcl::PointCloud<pcl::PointXYZ>::Ptr pcs = load_laser3d_frame("F:/data/3d-data/wangjinyi/map/Node3DDir-20181017T170336/LASER3D_0_0.bin");
//	//euclidean_cluster(pcs, "F:/data/tmp/4pcs_res/ec_clustered.vtk");
//	std::vector<gr::Point3D> set1;
//	load_laser3d_frame("F:/data/3d-data/wangjinyi/map/Node3DDir-20181017T170336/LASER3D_0_0.bin", set1);
//	remove_small_island(set1, 0.4, 200, "F:/data/tmp/4pcs_res/small_island_removed.vtk");
//	//image_based_segmentation(pcs, "F:/data/tmp/4pcs_res/ib_clustered.vtk");
//	return 0;
//}
