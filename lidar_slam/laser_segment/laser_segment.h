#ifndef LASER_SEGMENT_H
#define LASER_SEGMENT_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace zsw
{
	enum PointType
	{
		undefined = 0,
		ground = 0x01,
		edge = 0x02,
		plane = 0x04
	};

	std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<PointType>>
		segment_laser(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, const int ret_pc_types);
}

#endif //LASER_SEGMENT_H