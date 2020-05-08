#ifndef GROUND_SEG_H
#define GROUND_SEG_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <boost/multi_array.hpp>

struct ground_seg_params {
    // variables
    float rmax = 100;         // max radius of point to consider
    size_t max_bin_points = 200;  // max number of points to consider per bin
    size_t num_seed_points = 10;

    // for GP model
    float p_l = 4;  // length parameter, how close points have to be in the
    // GP model to correlate them
    float p_sf = 1;    // scaling on the whole covariance function
    float p_sn = 0.3;  // the expected noise for the mode
    float p_tmodel = 5;  // the required confidence required in order to consider
    // something ground
    float p_tdata = 5;  // scaled value that is required for a query point to be
    // considered ground
    float p_tg = 0.3;  // ground height threshold

    /// Height the robot (m), used to distinguish "drivable" overhanging points
    float robot_height = 1.2;

    // seeding parameters
    float max_seed_range = 50;   // meters
    float max_seed_height = 10;  // meters

    int num_bins_a = 72;
    int num_bins_l = 200;
};

struct range_height
{
    float height;
    float r;
    int index;
    bool is_ground;
};

class gaussian_process_ground_seg
{
public:
  gaussian_process_ground_seg(const ground_seg_params &params)
    : params_(params), rh_pts_(params.num_bins_a) {}

  /**
   * \brief segment point cloud, return labels, 1 for ground and 0 for nonground
   */
  std::vector<uint8_t> segment(pcl::PointCloud<pcl::PointXYZ>::Ptr &pts);
private:
  void gen_polar_bin_grid(pcl::PointCloud<pcl::PointXYZ>::Ptr &pts);
  void extract_seed(const int ind,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr &pts,
                    std::vector<range_height> &current_model,
                    std::vector<range_height> &sig_pts);
  Eigen::MatrixXd gp_kernel(const Eigen::VectorXd &x0, const Eigen::VectorXd &x1);
  ground_seg_params params_;
  std::vector<std::vector<range_height>> rh_pts_;
};
#endif //GROUND_SEG_H
