#ifndef FAST_RANGE_SEG_H
#define FAST_RANGE_SEG_H
#include <pcl/pcl_base.h>
#include <pcl/PointIndices.h>

namespace pcl {
	template <typename PointT>
	class fast_range_seg : public pcl::PCLBase<PointT>
	{
		typedef PCLBase<PointT> BasePCLBase;

	public:
		typedef pcl::PointCloud<PointT> PointCloud;
		typedef typename PointCloud::Ptr PointCloudPtr;
		typedef typename PointCloud::ConstPtr PointCloudConstPtr;

		typedef PointIndices::Ptr PointIndicesPtr;
		typedef PointIndices::ConstPtr PointIndicesConstPtr;

		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/** \brief Empty constructor. */
		fast_range_seg() : min_pts_per_cluster_(1),
			segment_theta_(1.0472),
			angle_bottom_(-15),
			ang_res_y_(2),
			ang_res_x_(0.2),
			v_scan_n_(16),
			h_scan_n_(1800)
		{};

		/** \brief Set the minimum number of points that a cluster needs to contain in order to be considered valid.
		  * \param[in] min_cluster_size the minimum cluster size
		  */
		inline void
			setMinClusterSize(int min_cluster_size)
		{
			min_pts_per_cluster_ = min_cluster_size;
		}

		/** \brief Get the minimum number of points that a cluster needs to contain in order to be considered valid. */
		inline int
			getMinClusterSize() const
		{
			return (min_pts_per_cluster_);
		}

		inline void set_seg_theta(const float seg_theta)
		{
			segment_theta_ = seg_theta;
		}

		inline float get_seg_theta()
		{
			return segment_theta_;
		}

		/** \brief Cluster extraction in a PointCloud given by <setInputCloud (), setIndices ()>
		  * \param[out] clusters the resultant point clusters
		  */
		void extract(std::vector<PointIndices> &clusters)
		{

		}

	protected:
		// Members derived from the base class
		using BasePCLBase::input_;
		using BasePCLBase::indices_;
		using BasePCLBase::initCompute;
		using BasePCLBase::deinitCompute;

		/** \brief The minimum number of points that a cluster needs to contain in order to be considered valid (default = 1). */
		int min_pts_per_cluster_;

		float segment_theta_;

		/** \brief lasser frame project into range image. */
		Eigen::MatrixXd range_img_;

		/** \brief index of point index in range_img_. */
		Eigen::MatrixXi ind_map_;

		float angle_bottom_;

		float ang_res_y_;

		float ang_res_x_;

		int v_scan_n_;

		int h_scan_n_;

		/** \brief Class getName method. */
		virtual std::string getClassName() const { return ("fast_range_extraction"); }

		/** \brief project the point cloud into range image. */
		void project_pc();
	};

	// template <typename PointT>
	// void fast_range_seg<PointT>::extract(std::vector<PointIndices>& clusters)
	// {
	// 	project_pc();
	// 	// for (size_t i = 0; i < N_SCAN; ++i)
	// 	// 	for (size_t j = 0; j < Horizon_SCAN; ++j)
	// 	// 		if (labelMat.at<int>(i, j) == 0)
	// 	// 			labelComponents(i, j);
	// 	//
	// 	// int sizeOfSegCloud = 0;
	// 	// for (size_t i = 0; i < N_SCAN; ++i) {
	// 	//
	// 	// 	segMsg.startRingIndex[i] = sizeOfSegCloud - 1 + 5;
	// 	//
	// 	// 	for (size_t j = 0; j < Horizon_SCAN; ++j) {
	// 	// 		if (labelMat.at<int>(i, j) > 0 || groundMat.at<int8_t>(i, j) == 1) {
	// 	// 			if (labelMat.at<int>(i, j) == 999999) {
	// 	// 				if (i > groundScanInd && j % 5 == 0) {
	// 	// 					outlierCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
	// 	// 					continue;
	// 	// 				}
	// 	// 				else {
	// 	// 					continue;
	// 	// 				}
	// 	// 			}
	// 	// 			if (groundMat.at<int8_t>(i, j) == 1) {
	// 	// 				if (j % 5 != 0 && j > 5 && j < Horizon_SCAN - 5)
	// 	// 					continue;
	// 	// 			}
	// 	// 			segMsg.segmentedCloudGroundFlag[sizeOfSegCloud] = (groundMat.at<int8_t>(i, j) == 1);
	// 	// 			segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;
	// 	// 			segMsg.segmentedCloudRange[sizeOfSegCloud] = rangeMat.at<float>(i, j);
	// 	// 			segmentedCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
	// 	// 			++sizeOfSegCloud;
	// 	// 		}
	// 	// 	}
	// 	//
	// 	// 	segMsg.endRingIndex[i] = sizeOfSegCloud - 1 - 5;
	// 	// }
	// 	//
	// 	// if (pubSegmentedCloudPure.getNumSubscribers() != 0) {
	// 	// 	for (size_t i = 0; i < N_SCAN; ++i) {
	// 	// 		for (size_t j = 0; j < Horizon_SCAN; ++j) {
	// 	// 			if (labelMat.at<int>(i, j) > 0 && labelMat.at<int>(i, j) != 999999) {
	// 	// 				segmentedCloudPure->push_back(fullCloud->points[j + i * Horizon_SCAN]);
	// 	// 				segmentedCloudPure->points.back().intensity = labelMat.at<int>(i, j);
	// 	// 			}
	// 	// 		}
	// 	// 	}
	// 	// }
	// }

	template <typename PointT>
	void fast_range_seg<PointT>::project_pc()
	{
		range_img_.resize(v_scan_n_, h_scan_n_);
		ind_map_.resize(v_scan_n_, h_scan_n_);
		ind_map_.setConstant(-1);
		range_img_.setZero();

		float verticalAngle, horizonAngle, range;
		size_t rowIdn, columnIdn, index, cloudSize;
		pcl::PointXYZ thisPoint;

		cloudSize = input_->points.size();
		for (size_t i = 0; i < cloudSize; ++i) {
			thisPoint.x = input_->points[i].x;
			thisPoint.y = input_->points[i].y;
			thisPoint.z = input_->points[i].z;
			verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
			rowIdn = (verticalAngle + angle_bottom_) / ang_res_y_;
			if (rowIdn < 0 || rowIdn >= v_scan_n_)
				continue;
			horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
			columnIdn = -round((horizonAngle - 90.0) / ang_res_x_) + h_scan_n_ / 2;
			if (columnIdn >= h_scan_n_)
				columnIdn -= h_scan_n_;
			if (columnIdn < 0 || columnIdn >= h_scan_n_)
				continue;
			range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
			range_img_(rowIdn, columnIdn) = range;
			ind_map_(rowIdn, columnIdn) = i;
		}
	}
}

#endif //FAST_RANGE_IMAGE_SEGMENTATION_H
