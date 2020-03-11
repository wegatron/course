#include "imls_icp.h"
#include <csm/csm_all.h>
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "champion_nav_msgs/ChampionNavLaserScan.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

//pcl::visualization::CloudViewer g_cloudViewer("cloud_viewer");
//此处bag包的地址需要自行修改
std::string bagfile = "./src/bag/imls_icp.bag";

class imlsDebug
{
public:
  imlsDebug()
  {
    m_imlsPathPub = m_node.advertise<nav_msgs::Path>("imls_path_pub_",1,true);
    m_imlsPath.header.stamp = ros::Time::now();
    m_imlsPath.header.frame_id = "odom";
    m_odomPathPub = m_node.advertise<nav_msgs::Path>("odom_path_pub_",1,true);
    m_odomPath.header.stamp = ros::Time::now();
    m_odomPath.header.frame_id = "odom";


    // wegatron add csm publisher
    m_csmPathPub = m_node.advertise<nav_msgs::Path>("csm_path_pub_",1,true);
    m_csmPath.header.stamp = ros::Time::now();
    m_csmPath.header.frame_id = "odom";
    m_prevScan = nullptr;

    m_isFirstFrame = true;


    m_PIICPParams.min_reading = 0.1;
    m_PIICPParams.max_reading = 20;

    //设置位姿最大的变化范围
    m_PIICPParams.max_angular_correction_deg = 20.0;
    m_PIICPParams.max_linear_correction = 1;

    //设置迭代停止的条件
    m_PIICPParams.max_iterations = 50;
    m_PIICPParams.epsilon_xy = 0.000001;
    m_PIICPParams.epsilon_theta = 0.0000001;

    //设置correspondence相关参数
    m_PIICPParams.max_correspondence_dist = 1;
    m_PIICPParams.sigma = 0.01;
    m_PIICPParams.use_corr_tricks = 1;

    //设置restart过程，因为不需要restart所以可以不管
    m_PIICPParams.restart = 0;
    m_PIICPParams.restart_threshold_mean_error = 0.01;
    m_PIICPParams.restart_dt = 1.0;
    m_PIICPParams.restart_dtheta = 0.1;

    //设置聚类参数
    m_PIICPParams.clustering_threshold = 0.2;

    //用最近的10个点来估计方向
    m_PIICPParams.orientation_neighbourhood = 10;

    //设置使用PI-ICP
    m_PIICPParams.use_point_to_line_distance = 1;

    //不进行alpha_test
    m_PIICPParams.do_alpha_test = 0;
    m_PIICPParams.do_alpha_test_thresholdDeg = 5;

    //设置trimmed参数 用来进行outlier remove
    m_PIICPParams.outliers_maxPerc = 0.9;
    m_PIICPParams.outliers_adaptive_order = 0.7;
    m_PIICPParams.outliers_adaptive_mult = 2.0;

    //进行visibility_test 和 remove double
    m_PIICPParams.do_visibility_test = 1;
    m_PIICPParams.outliers_remove_doubles = 1;
    m_PIICPParams.do_compute_covariance = 0;
    m_PIICPParams.debug_verify_tricks = 0;
    m_PIICPParams.use_ml_weights = 0;
    m_PIICPParams.use_sigma_weights = 0;

    rosbag::Bag bag;
    bag.open(bagfile, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("/sick_scan"));
    topics.push_back(std::string("/odom"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    //按顺序读取bag内激光的消息和里程计的消息
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
      {
        champion_nav_msgs::ChampionNavLaserScanConstPtr scan = m.instantiate<champion_nav_msgs::ChampionNavLaserScan>();
        if(scan != NULL)
          championLaserScanCallback(scan);

        nav_msgs::OdometryConstPtr odom = m.instantiate<nav_msgs::Odometry>();
        if(odom != NULL)
          odomCallback(odom);

        ros::spinOnce();
        if(!ros::ok())
          break;
      }
    // m_laserscanSub = m_nh.subscribe("sick_scan",5,&imlsDebug::championLaserScanCallback,this);
  }

    //将激光消息转换为激光坐标系下的二维点云
    void ConvertChampionLaserScanToEigenPointCloud(const champion_nav_msgs::ChampionNavLaserScanConstPtr& msg,
                                                   std::vector<Eigen::Vector2d>& eigen_pts)
    {

        eigen_pts.clear();
        for(int i = 0; i < msg->ranges.size(); ++i)
        {
            if(msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max)
                continue;

            double lx = msg->ranges[i] * std::cos(msg->angles[i]);
            double ly = msg->ranges[i] * std::sin(msg->angles[i]);

            if(std::isnan(lx) || std::isinf(ly) ||
               std::isnan(ly) || std::isinf(ly))
                continue;

            eigen_pts.push_back(Eigen::Vector2d(lx,ly));
        }
    }

    void eigen_pts2ldp(const std::vector<Eigen::Vector2d>& eigen_pts, LDP &ldp)
    {
        if(ldp != nullptr) delete ldp;
        ldp = ld_alloc_new(eigen_pts.size());
        for(int i=0; i<eigen_pts.size(); ++i)
        {
            ldp->valid[i] = 1;
            ldp->readings[i] = eigen_pts[i].norm();
            ldp->theta[i] = atan2(eigen_pts[i].y(), eigen_pts[i].x());
        }

        // ??????
        ldp->min_theta = ldp->theta[0];
        ldp->max_theta = ldp->theta[eigen_pts.size()-1];

        ldp->odometry[0] = 0.0;
        ldp->odometry[1] = 0.0;
        ldp->odometry[2] = 0.0;

        ldp->true_pose[0] = 0.0;
        ldp->true_pose[1] = 0.0;
        ldp->true_pose[2] = 0.0;

        ldp->estimate[0] = 0.0;
        ldp->estimate[1] = 0.0;
        ldp->estimate[2] = 0.0;
    }

    void LaserScanToLDP(const champion_nav_msgs::ChampionNavLaserScanConstPtr& pScan,
                               LDP& ldp)
    {
#if 1
        int nPts = pScan->ranges.size();
        ldp = ld_alloc_new(nPts);
        for(int i = 0;i < nPts;i++)
        {
            double dist = pScan->ranges[i];
            if(dist > 0.1 && dist < 100)
            {
                ldp->valid[i] = 1;
                ldp->readings[i] = dist;
                ldp->theta[i] = pScan->angles[i];
            }
            else
            {
                ldp->valid[i] = 0;
                ldp->readings[i] = 0;
                ldp->theta[i] = 0;
            }
        }

#else
        int nPts = 100;
        ldp = ld_alloc_new(nPts);
        for(int i = 0;i < nPts;i++)
        {
            ldp->valid[i] = 1;
            ldp->readings[i] = i;
            ldp->theta[i] = -4.1415;
        }
#endif
        ldp->min_theta = ldp->theta[0];
        ldp->max_theta = ldp->theta[nPts-1];

        ldp->odometry[0] = 0.0;
        ldp->odometry[1] = 0.0;
        ldp->odometry[2] = 0.0;

        ldp->true_pose[0] = 0.0;
        ldp->true_pose[1] = 0.0;
        ldp->true_pose[2] = 0.0;


        ldp->estimate[0] = 0.0;
        ldp->estimate[1] = 0.0;
        ldp->estimate[2] = 0.0;
    }

  void championLaserScanCallback(const champion_nav_msgs::ChampionNavLaserScanConstPtr& msg)
  {
    if(m_isFirstFrame == true)
      {
        std::cout <<"First Frame"<<std::endl;
        m_isFirstFrame = false;
        m_prevLaserPose = Eigen::Vector3d(0, 0, 0);
        pubPath(m_prevLaserPose, m_imlsPath, m_imlsPathPub);
        ConvertChampionLaserScanToEigenPointCloud(msg, m_prevPointCloud);
        //LaserScanToLDP(msg, m_prevScan);
        m_prev_csm_pose = Eigen::Vector3d(0,0,0);
        pubPath(m_prev_csm_pose, m_csmPath, m_csmPathPub);
        eigen_pts2ldp(m_prevPointCloud, m_prevScan);
        return ;
      }

    std::vector<Eigen::Vector2d> nowPts;
    LDP cur_scan = nullptr;        ConvertChampionLaserScanToEigenPointCloud(msg, nowPts);
    //LaserScanToLDP(msg, cur_scan);
    eigen_pts2ldp(m_prevPointCloud, cur_scan);

    //调用imls进行icp匹配，并输出结果．
    m_imlsMatcher.setSourcePointCloud(nowPts);
    m_imlsMatcher.setTargetPointCloud(m_prevPointCloud);

    std::cout << "-----" << std::endl;
    Eigen::Matrix3d rPose,rCovariance;
    if(m_imlsMatcher.Match(rPose,rCovariance))
      {
        std::cout <<"IMLS Match Successful:"<<rPose(0,2)<<","<<rPose(1,2)<<","<<atan2(rPose(1,0),rPose(0,0))*57.295<<std::endl;
        Eigen::Matrix3d lastPose;
        lastPose << cos(m_prevLaserPose(2)), -sin(m_prevLaserPose(2)), m_prevLaserPose(0),
          sin(m_prevLaserPose(2)),  cos(m_prevLaserPose(2)), m_prevLaserPose(1),
          0, 0, 1;
        Eigen::Matrix3d nowPose = lastPose * rPose;
        m_prevLaserPose << nowPose(0, 2), nowPose(1, 2), atan2(nowPose(1,0), nowPose(0,0));
        pubPath(m_prevLaserPose, m_imlsPath, m_imlsPathPub);
      }
    else
      {
        std::cout <<"IMLS Match Failed!!!!"<<std::endl;
      }
    std::cout << "-----" << std::endl;
 #if 1
    // csm macher
    {
      m_PIICPParams.laser_ref = m_prevScan;
      m_PIICPParams.laser_sens = cur_scan;

      m_PIICPParams.first_guess[0] = 0;
      m_PIICPParams.first_guess[1] = 0;
      m_PIICPParams.first_guess[2] = 0;

      sm_result output_res;
      output_res.cov_x_m = 0;
      output_res.dx_dy1_m = 0;
      output_res.dx_dy2_m = 0;
      sm_icp(&m_PIICPParams, &output_res);
      //nowPose在lastPose中的坐标
      Eigen::Vector3d res_pose;
      if(output_res.valid)
        {
          // publish path
          res_pose[0] = output_res.x[0];
          res_pose[1] = output_res.x[1];
          res_pose[2] = output_res.x[2];

          std::cout << "PI ICP Success:" << res_pose.transpose() << std::endl;
          Eigen::Matrix3d lastPose;
          lastPose << cos(m_prev_csm_pose(2)), -sin(m_prev_csm_pose(2)), m_prev_csm_pose(0),
            sin(m_prev_csm_pose(2)),  cos(m_prev_csm_pose(2)), m_prev_csm_pose(1),
            0, 0, 1;

          Eigen::Matrix3d rPose;
          rPose << cos(res_pose(2)), -sin(res_pose(2)), res_pose(0),
            sin(res_pose(2)),  cos(res_pose(2)), res_pose(1),
            0, 0, 1;
          Eigen::Matrix3d nowPose = lastPose * rPose;
          m_prev_csm_pose << nowPose(0, 2), nowPose(1, 2), atan2(nowPose(1,0), nowPose(0,0));
          pubPath(m_prev_csm_pose, m_csmPath, m_csmPathPub);
        }
      else {
        std::cout <<"PI ICP Failed!!!!!!!"<<std::endl;
      }
      delete m_prevScan;
      m_prevScan = cur_scan;

    }
#endif
    m_prevPointCloud = nowPts;
  }

    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        if(m_isFirstFrame == true)
            return;

        pubPath(msg, m_odomPath, m_odomPathPub);
    }

    //发布路径消息
    void pubPath(Eigen::Vector3d& pose, nav_msgs::Path &path, ros::Publisher &mcu_path_pub_)
    {
        ros::Time current_time = ros::Time::now();
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = pose(0);
        this_pose_stamped.pose.position.y = pose(1);

        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(pose(2));
        this_pose_stamped.pose.orientation.x = goal_quat.x;
        this_pose_stamped.pose.orientation.y = goal_quat.y;
        this_pose_stamped.pose.orientation.z = goal_quat.z;
        this_pose_stamped.pose.orientation.w = goal_quat.w;

        this_pose_stamped.header.stamp = current_time;
        this_pose_stamped.header.frame_id = "odom";
        path.poses.push_back(this_pose_stamped);
        mcu_path_pub_.publish(path);
    }

    void pubPath(const nav_msgs::OdometryConstPtr& msg, nav_msgs::Path &path, ros::Publisher &mcu_path_pub_)
    {
        ros::Time current_time = ros::Time::now();
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = msg->pose.pose.position.x;
        this_pose_stamped.pose.position.y = msg->pose.pose.position.y;

        this_pose_stamped.pose.orientation.x = msg->pose.pose.orientation.x;
        this_pose_stamped.pose.orientation.y = msg->pose.pose.orientation.y;
        this_pose_stamped.pose.orientation.z = msg->pose.pose.orientation.z;
        this_pose_stamped.pose.orientation.w = msg->pose.pose.orientation.w;

        this_pose_stamped.header.stamp = current_time;
        this_pose_stamped.header.frame_id = "odom";
        path.poses.push_back(this_pose_stamped);
        mcu_path_pub_.publish(path);
    }

    bool m_isFirstFrame;
    ros::NodeHandle m_nh;
    IMLSICPMatcher m_imlsMatcher;
    Eigen::Vector3d m_prevLaserPose;
    std::vector<Eigen::Vector2d> m_prevPointCloud;

    nav_msgs::Path m_imlsPath;
    nav_msgs::Path m_odomPath;

    tf::TransformListener m_tfListener;
    ros::NodeHandle m_node;

    ros::Subscriber m_laserscanSub;
    ros::Publisher m_imlsPathPub;
    ros::Publisher m_odomPathPub;

    // wegatron add for csm
    sm_params m_PIICPParams;
    LDP m_prevScan;
    Eigen::Vector3d m_prev_csm_pose;
    nav_msgs::Path m_csmPath;
    ros::Publisher m_csmPathPub;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "imls_debug");

    imlsDebug imls_debug;

    ros::spin();

    return (0);
}

