#include "demo_io.h"

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkFloatArray.h>
#include <vtkDoubleArray.h>
#include <vtkIntArray.h>
#include <vtkCellData.h>
#include <vtkPointData.h>
#include <vtkPolyDataWriter.h>
#include <lasser3d_io.h>
#include <pcl/common/transforms.h>

bool read_point_cloud(const std::string& file_path, std::vector<gr::Point3D>& pts)
{
	pts.clear();
	std::ifstream cloud_file;
	cloud_file.open(file_path, std::ios::binary);
	if (cloud_file.is_open() == false)
	{
		std::cout << "open cloud file fail!" << std::endl;
		return false;
	}

	std::string point_str;
	bool if_get_binary_flag = false;
	while (getline(cloud_file, point_str))
	{
		if (point_str == "#cloud_data_binary")
		{
			if_get_binary_flag = true;
			break;
		}
	}

	if (if_get_binary_flag == false)
	{
		cloud_file.close();
		std::cout << "read cloud file binary " << file_path << " fail!" << std::endl;
		return false;
	}

	while (true)
	{
		float x, y, z;
		unsigned char rssi;
		cloud_file.read((char*)(&x), sizeof(float));
		if (cloud_file.gcount() < sizeof(float))
		{
			break;
		}
		cloud_file.read((char*)(&y), sizeof(float));
		if (cloud_file.gcount() < sizeof(float))
		{
			break;
		}
		cloud_file.read((char*)(&z), sizeof(float));
		if (cloud_file.gcount() < sizeof(float))
		{
			break;
		}
		cloud_file.read((char*)(&rssi), sizeof(unsigned char));
		if (cloud_file.gcount() < sizeof(unsigned char))
		{
			break;
		}
		gr::Point3D point;
		point.x() = x;
		point.y() = y;
		point.z() = z;
		pts.push_back(point);
	}

	cloud_file.close();


	std::cout << "read cloud file binary " << file_path << " success! point size " << pts.size() << std::endl;
	return true;
}

bool load_laser3d_frame(const std::string& laser3d_file, std::vector<gr::Point3D>& pts)
{
	int beam_size = 0;
	int seg_size = 0;
	std::ifstream ifs(laser3d_file, std::ifstream::binary);
	if (!ifs)
	{
		std::cerr << "unable to read file:" << laser3d_file << std::endl;
		return false;
	}
	pts.clear();
	boost::regex word_regex("\\s+");
	while (true)
	{
		std::string tmp_str;
		std::getline(ifs, tmp_str);
		std::vector<std::string> fields;
		regex_split(std::back_inserter(fields), tmp_str, word_regex);
		if (fields[0] == "#segment_beam_size")
		{
			beam_size = std::stoi(fields[1]);
		}
		else if (fields[0] == "#segment_size")
		{
			seg_size = std::stoi(fields[1]);
		}
		else if (fields[0] == "#data_binary")
			break;
	}
	int seg_index = -1;
	gr::Point3D tmp_pt;
	char buff[10];
	while (true)
	{
		if (++seg_index > seg_size) break;
		ifs.read(buff, sizeof(float)); // read time
		for (int i = 0; i < beam_size; ++i)
		{
			// pts_cloud->insert()
			ifs.read(reinterpret_cast<char*>(&tmp_pt.x()), sizeof(float));
			ifs.read(reinterpret_cast<char*>(&tmp_pt.y()), sizeof(float));
			ifs.read(reinterpret_cast<char*>(&tmp_pt.z()), sizeof(float));
			ifs.read(&buff[0], 1); // is valid
			ifs.read(&buff[1], 1); // remission
			// if (buff[0] == 0) continue;
			float sqn = tmp_pt.pos().squaredNorm();
			if(sqn > 0.0625 || sqn < 100) pts.push_back(tmp_pt);
		}
	}
	return true;
}

bool write_vtk(const std::string& file_path, const std::vector<gr::Point3D>& pts)
{
	vtkSmartPointer<vtkPoints> points_data = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPolyData> vtk_points_cloud = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkFloatArray> index = vtkSmartPointer<vtkFloatArray>::New();
	vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
	index->SetName("index");
	int cur_index = 0;

	for (const auto& point : pts)
	{
		auto id = points_data->InsertNextPoint(point.x(), point.y(), point.z());
		vertices->InsertNextCell(1);
		vertices->InsertCellPoint(id);
		index->InsertNextTuple1(cur_index);
	}
	++cur_index;
	vtk_points_cloud->SetPoints(points_data);
	vtk_points_cloud->SetVerts(vertices);
	if (cur_index > 1) vtk_points_cloud->GetPointData()->SetScalars(index);

	// write out
	static vtkSmartPointer<vtkPolyDataWriter> poly_data_writer = vtkSmartPointer<vtkPolyDataWriter>::New();
	poly_data_writer->SetInputData(vtk_points_cloud);
	poly_data_writer->SetFileName(file_path.c_str());
	poly_data_writer->SetFileTypeToBinary();
	return poly_data_writer->Write();
}

bool write_vtk2(const std::string& file_path, const std::vector<gr::Point3D>& pts_a,
                const std::vector<gr::Point3D>& pts_b)
{
	vtkSmartPointer<vtkPoints> points_data = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPolyData> vtk_points_cloud = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkIntArray> index = vtkSmartPointer<vtkIntArray>::New();
	vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
	index->SetName("index");
	int cur_index = 0;

	for (const auto& point : pts_a)
	{
		auto id = points_data->InsertNextPoint(point.x(), point.y(), point.z());
		vertices->InsertNextCell(1);
		vertices->InsertCellPoint(id);
		index->InsertNextTuple1(cur_index);
	}
	++cur_index;

	for (const auto& point : pts_b)
	{
		auto id = points_data->InsertNextPoint(point.x(), point.y(), point.z());
		vertices->InsertNextCell(1);
		vertices->InsertCellPoint(id);
		index->InsertNextTuple1(cur_index);
	}

	vtk_points_cloud->SetPoints(points_data);
	vtk_points_cloud->SetVerts(vertices);
	if (cur_index > 0) vtk_points_cloud->GetPointData()->SetScalars(index);

	// write out
	static vtkSmartPointer<vtkPolyDataWriter> poly_data_writer = vtkSmartPointer<vtkPolyDataWriter>::New();
	poly_data_writer->SetInputData(vtk_points_cloud);
	poly_data_writer->SetFileName(file_path.c_str());
	poly_data_writer->SetFileTypeToBinary();
	return poly_data_writer->Write();
}

bool rt2vtk(const std::string& file_path,
	const std::vector<Eigen::Matrix<float, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<float, 6, 1>>>& rts)
{
	vtkSmartPointer<vtkPoints> points_data = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPolyData> vtk_points_cloud = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
	vtkSmartPointer<vtkFloatArray> rotation = vtkSmartPointer<vtkFloatArray>::New();
	rotation->SetNumberOfComponents(9);
	rotation->SetNumberOfTuples(rts.size());
	rotation->SetName("rotation");
	auto len = rts.size();
	for (auto i = 0; i < len; ++i)
	{
		const auto id = points_data->InsertNextPoint(rts[i].x(), rts[i].y(), rts[i].z());
		vertices->InsertNextCell(1);
		vertices->InsertCellPoint(id);
		rotation->InsertTuple(i, rts[i].data()+3);
	}
	vtk_points_cloud->SetPoints(points_data);
	vtk_points_cloud->SetVerts(vertices);
	vtk_points_cloud->GetPointData()->AddArray(rotation);
	// write out
	static vtkSmartPointer<vtkPolyDataWriter> poly_data_writer = vtkSmartPointer<vtkPolyDataWriter>::New();
	poly_data_writer->SetInputData(vtk_points_cloud);
	poly_data_writer->SetFileName(file_path.c_str());
	poly_data_writer->SetFileTypeToBinary();
	return poly_data_writer->Write();
	return true;
}

bool load_local_map(const std::string& graph_file, const Eigen::Vector3f& pos, std::vector<gr::Point3D>& pts)
{
	std::cout << "pos:" << pos.transpose() << std::endl;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> laser_pcs;
	std::vector<Eigen::Affine3f> transforms;
	zsw_gz_common::load_graph_nodes(graph_file, laser_pcs, transforms);
	pts.clear();
	// pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_out(new pcl::PointCloud<pcl::PointXYZ>);
	for(int i = 0; i<laser_pcs.size(); ++i)
	{
		// if scan close to pos
		if ((pos - transforms[i].translation()).squaredNorm() > 100) continue;
		for(const auto &pt : laser_pcs[i]->points)
		{
			Eigen::Vector3f ptv(pt.x, pt.y, pt.z);
			if (ptv.squaredNorm() < 0.0625)
				continue;
			Eigen::Vector3f tr_pt = transforms[i] * ptv;
			if ((pos - tr_pt).squaredNorm() < 100)
				pts.emplace_back(tr_pt[0], tr_pt[1], tr_pt[2]);
		}
	}
	return true;
}
