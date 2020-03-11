#ifndef SURFEL_H
#define SURFEL_H

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace zsw
{

	struct surfel
	{
		Eigen::Matrix<double, 6, 1> des; //!< surfel descriptor. [0~2] u, center position, [3~5] p * v, v is eigen vector of the smallest eigen value(normal of the plane).

		// for visualization
		Eigen::Vector3d eigen_value; //!< eigen value in increasing order
		Eigen::Matrix3d eigen_vector; //!< each column is one eigen vector(normalized), correspond to eigen value
#if ZSW_DEBUG
		Eigen::Matrix3d raw; //!< \sum_i p_ip_i^T - \mu \mu_i^T
#endif
	};

	/**
	 * \brief extract single level of surfel feature
	 * \param pc input point cloud
	 * \param min_pos, bounding box, for offset exactly
	 * \param max_pos, bounding box, for offset exactly
	 * \param cube_len cube lenth
	 * \param min_pts_percell the minimum number of points in grid, to extract feature.
	 * \return features.
	 */
	std::vector<surfel> extract_surfels_basic(pcl::PointCloud<pcl::PointXYZ>::Ptr &pc,
		const pcl::PointXYZ &min_pt,
		const pcl::PointXYZ &max_pt,
		const float cube_len,
		const size_t min_pts_percell=30);

	std::vector<surfel> extract_surfel_multi_resolution(pcl::PointCloud<pcl::PointXYZ>::Ptr &pc, const float cube_len, 
		const float scale, const int num_layer);


}

#endif SURFEL_H