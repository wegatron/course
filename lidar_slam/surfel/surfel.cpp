#include "surfel.h"
#include <pcl/common/common.h>
#include <boost/multi_array.hpp>


std::vector<zsw::surfel> zsw::extract_surfels_basic(pcl::PointCloud<pcl::PointXYZ>::Ptr& pc,
	const pcl::PointXYZ &min_pt,
	const pcl::PointXYZ &max_pt,
	const float cube_len, const size_t min_pts_percell)
{
	//pcl::PointXYZ min_pt, max_pt;
	//pcl::getMinMax3D(*pc, min_pt, max_pt);
	int grid_size[3] = {
		 ceil((max_pt.x - min_pt.x) / cube_len),
		 ceil((max_pt.y - min_pt.y) / cube_len),
		 ceil((max_pt.z - min_pt.z) / cube_len)
	};

	boost::multi_array<Eigen::Matrix3d, 3, Eigen::aligned_allocator<Eigen::Matrix3d>> feature_mat{boost::extents[grid_size[0]][grid_size[1]][grid_size[2]]};
	boost::multi_array<Eigen::Vector3d, 3, Eigen::aligned_allocator<Eigen::Vector3d>> mu{ boost::extents[grid_size[0]][grid_size[1]][grid_size[2]] }; // center of each bin
	boost::multi_array<size_t, 3> pt_count{ boost::extents[grid_size[0]][grid_size[1]][grid_size[2]] };

	for (auto i = 0; i < grid_size[0]; ++i)
	{
		for (auto j = 0; j < grid_size[1]; ++j)
		{
			for (auto k = 0; k < grid_size[2]; ++k)
			{
				memset(feature_mat[i][j][k].data(), 0, sizeof(double) * 9);
				memset(mu[i][j][k].data(), 0, sizeof(double) * 3);
				pt_count[i][j][k] = 0;
			}
		}
	}

	for(const auto &pt : pc->points)
	{
		int index[3] = {
			(pt.x - min_pt.x) / cube_len,
			(pt.y - min_pt.y) / cube_len,
			(pt.z - min_pt.z) / cube_len
		};
		Eigen::Vector3d p(pt.x, pt.y, pt.z);
		feature_mat[index[0]][index[1]][index[2]] += p * p.transpose();
		mu[index[0]][index[1]][index[2]] += p;
		++pt_count[index[0]][index[1]][index[2]];
	}
	std::vector<zsw::surfel> surfel_features;
	surfel_features.reserve(grid_size[0] * grid_size[1] * grid_size[2]);
	for(auto i = 0; i < grid_size[0]; ++i)
	{
		for(auto j=0; j<grid_size[1]; ++j)
		{
			for(auto k=0; k<grid_size[2]; ++k)
			{
				const size_t cnt = pt_count[i][j][k];
				double inv_cnt = 1.0 / cnt;
				if (pt_count[i][j][k] < min_pts_percell) continue;
				auto cur_mu = mu[i][j][k] * inv_cnt;
				Eigen::Matrix3d e = feature_mat[i][j][k] * inv_cnt  - cur_mu * cur_mu.transpose();
				Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(e);
				if (Eigen::Success != eigen_solver.info()) continue;
				surfel_features.emplace_back();
				surfel &tmp_surfel = surfel_features.back();
#if ZSW_DEBUG
				tmp_surfel.raw = e;
#endif
				tmp_surfel.eigen_value = eigen_solver.eigenvalues();
				tmp_surfel.eigen_vector = eigen_solver.eigenvectors();
				assert(tmp_surfel.eigen_value[0] >= 0);
				tmp_surfel.des.block<3, 1>(0, 0) = cur_mu;
				double p = 2 * (tmp_surfel.eigen_value[1] - tmp_surfel.eigen_value[0]) / (tmp_surfel.eigen_value[2] + tmp_surfel.eigen_value[1] + tmp_surfel.eigen_value[0]);
				tmp_surfel.des.block<3, 1>(3, 0) = p * tmp_surfel.eigen_vector.block<3, 1>(0, 0);
			}
		}
	}

	return surfel_features;
}

std::vector<zsw::surfel> zsw::extract_surfel_multi_resolution(pcl::PointCloud<pcl::PointXYZ>::Ptr& pc,
	const float cube_len, const float scale, const int num_layer)
{
	pcl::PointXYZ min_pt, max_pt;
	pcl::getMinMax3D(*pc, min_pt, max_pt);

	const float inv_scale = 1.0 / scale;
	double cur_cube_len = cube_len;
	double half_cube_len = cube_len * 0.5;

	pcl::PointXYZ start_pos;
	for(int i=0; i<num_layer; ++i)
	{
		extract_surfels_basic(pc, min_pt, max_pt, cur_cube_len);
		
		start_pos = { static_cast<float>(min_pt.x+ half_cube_len), min_pt.y, min_pt.z };
		extract_surfels_basic(pc, start_pos, max_pt, cur_cube_len);

		start_pos = { min_pt.x, static_cast<float>(min_pt.y + half_cube_len), min_pt.z };
		extract_surfels_basic(pc, start_pos, max_pt, cur_cube_len);

		start_pos = { static_cast<float>(min_pt.x + half_cube_len), static_cast<float>(min_pt.y + half_cube_len), min_pt.z };
		extract_surfels_basic(pc, start_pos, max_pt, cur_cube_len);

		start_pos = { min_pt.x , min_pt.y , static_cast<float>(min_pt.z+half_cube_len) };
		extract_surfels_basic(pc, min_pt, max_pt, cur_cube_len);

		start_pos = { static_cast<float>(min_pt.x + half_cube_len), min_pt.y, static_cast<float>(min_pt.z + half_cube_len) };
		extract_surfels_basic(pc, start_pos, max_pt, cur_cube_len);

		start_pos = { min_pt.x, static_cast<float>(min_pt.y + half_cube_len), static_cast<float>(min_pt.z + half_cube_len) };
		extract_surfels_basic(pc, start_pos, max_pt, cur_cube_len);

		start_pos = { static_cast<float>(min_pt.x + half_cube_len), static_cast<float>(min_pt.y + half_cube_len), static_cast<float>(min_pt.z + half_cube_len) };
		extract_surfels_basic(pc, start_pos, max_pt, cur_cube_len);

		cur_cube_len *= inv_scale;
		half_cube_len = cube_len * 0.5;
	}
	return {};
}
