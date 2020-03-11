#include "gaussian_newton_method.h"

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Path.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

double GN_NormalizationAngle(double angle);

class GaussianNewtonDebug
{
public:
    GaussianNewtonDebug()
    {
        m_laserscanSub = m_nh.subscribe("sick_scan",5,&GaussianNewtonDebug::rosLaserScanCallback,this);

        m_odomPub = m_nh.advertise<nav_msgs::Path>("odom_path",1,true);

        m_gaussianNewtonPub = m_nh.advertise<nav_msgs::Path>("gaussian_newton_path",1,true);
        m_pcsPub = m_nh.advertise<sensor_msgs::PointCloud2>("/pcs", 1);
    }

    //单纯的数据类型转换，不进行坐标系转换．
    void ConvertLaserScanToEigenPointCloud(const sensor_msgs::LaserScanConstPtr& msg,
                                                   std::vector<Eigen::Vector2d>& eigen_pts)
    {
        eigen_pts.clear();
        for(int i = 0; i < msg->ranges.size();i++)
        {
            if(msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max)
                continue;

            double angle = msg->angle_min + msg->angle_increment * i;

            double lx = msg->ranges[i] * std::cos(angle);
            double ly = msg->ranges[i] * std::sin(angle);

            if(std::isnan(lx) || std::isinf(ly)||
               std::isnan(ly) || std::isinf(ly))
                continue;

            eigen_pts.push_back(Eigen::Vector2d(lx,ly));
        }
    }

    void PublishPath(ros::Publisher& puber,
                     std::vector<Eigen::Vector3d>& path)
    {
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "/odom";

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "/odom";
        for(int i = 0; i < path.size();i++)
        {
            Eigen::Vector3d traj_node = path[i];
            pose.pose.position.x = traj_node(0);
            pose.pose.position.y = traj_node(1);
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(traj_node(2));
            path_msg.poses.push_back(pose);
        }

        puber.publish(path_msg);
    }

    void rosLaserScanCallback(const sensor_msgs::LaserScanConstPtr& msg)
    {
        static bool isFirstFrame = true;
        Eigen::Vector3d nowPose;
        if(getLaserPose(msg->header.stamp,nowPose) == false)
        {
            std::cout <<"Failed to get Laser Pose"<<std::endl;
            return ;
        }

        if(isFirstFrame == true)
        {
            std::cout <<"First Frame"<<std::endl;
            isFirstFrame = false;

            m_prevLaserPose = nowPose;
            ConvertLaserScanToEigenPointCloud(msg,m_prevPts);

            m_odomPath.push_back(nowPose);
            m_gaussianNewtonPath.push_back(nowPose);

            return ;
        }

        auto pre_odom_pose = m_odomPath.back();
        double delta_dist2 = std::pow(nowPose(0) - pre_odom_pose(0),2) + std::pow(nowPose(1) - pre_odom_pose(1),2);
        double delta_angle = std::fabs(tfNormalizeAngle(nowPose(2) - pre_odom_pose(2)));

        if(delta_dist2 < 0.2 * 0.2 &&
           delta_angle < tfRadians(10.0))
        {
            return ;
        }
        //数据类型转换．
        std::vector<Eigen::Vector2d> nowPts;
        ConvertLaserScanToEigenPointCloud(msg,nowPts);
        //生成地图
        map_t* map = CreateMapFromLaserPoints(m_prevLaserPose,m_prevPts,0.1);

        //进行优化．
        //初始解为上一帧激光位姿+运动增量
        Eigen::Vector3d deltaPose = nowPose - m_odomPath.back();
        deltaPose(2) = GN_NormalizationAngle(deltaPose(2));

        Eigen::Matrix3d R_laser;
        double theta = m_prevLaserPose(2);
        R_laser << cos(theta), -sin(theta), 0,
                   sin(theta),  cos(theta), 0,
                        0,          0,      1;

        Eigen::Matrix3d R_odom;
        theta = m_odomPath.back()(2);
        R_odom << cos(theta), -sin(theta), 0, 
                  sin(theta),  cos(theta), 0,
                       0,          0,      1;

        // 加上变化的x,y和角度: delta_pose是相对于上一个odom的变化量, 需要转换为基于上一帧激光的变化量
        Eigen::Vector3d finalPose = m_prevLaserPose + R_laser * R_odom.transpose() * deltaPose;
        finalPose(2) = GN_NormalizationAngle(finalPose(2));

        std::cout << "Init Pose:" << finalPose.transpose() << std::endl;
        GaussianNewtonOptimization(map,finalPose,nowPts);

        //更新数据．
        m_prevLaserPose = finalPose;
        m_prevPts = nowPts;

        std::cout <<"Final Pose:"<<finalPose.transpose()<<std::endl<< std::endl;


        //释放地图
        map_free(map);

        //保存路径．
        m_odomPath.push_back(nowPose);
        m_gaussianNewtonPath.push_back(finalPose);

        PublishPath(m_odomPub,m_odomPath);
        PublishPath(m_gaussianNewtonPub,m_gaussianNewtonPath);

        // publish point clouds
        Eigen::Affine2d rt(Eigen::Translation2d(finalPose[0], finalPose[1]) * Eigen::Rotation2Dd(finalPose[2]));
        static pcl::PointCloud<pcl::PointXYZ>::Ptr pcs(new pcl::PointCloud<pcl::PointXYZ>);
        for(const auto &pt : nowPts) {
          Eigen::Vector2d tr_pt = rt * pt;
          pcs->push_back(pcl::PointXYZ(tr_pt.x(), tr_pt.y(), 0));
        }

        // do filtering
        std::cout << "num pc:" << pcs->size() << std::endl;
        sensor_msgs::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(*pcs, laserCloudTemp);
        laserCloudTemp.header.stamp = ros::Time::now();
        laserCloudTemp.header.frame_id = "/odom";
        m_pcsPub.publish(laserCloudTemp);
    }

    bool getLaserPose(ros::Time t,
                     Eigen::Vector3d& pose)
    {
        // Get the robot's pose
        tf::Stamped<tf::Pose> ident (tf::Transform(tf::createQuaternionFromRPY(0,0,0),
                                                   tf::Vector3(0,0,0)), t, "/base_laser");
        tf::Stamped<tf::Transform> odom_pose;
        try
        {
            m_tfListener.transformPose("/odom", ident, odom_pose);
        }
        catch(tf::TransformException e)
        {
            ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
            return false;
        }

        double yaw = tf::getYaw(odom_pose.getRotation());
        pose << odom_pose.getOrigin().x(),odom_pose.getOrigin().y(),yaw;

        return true;
    }

    ros::NodeHandle m_nh;

    Eigen::Vector3d m_prevLaserPose;

    std::vector<Eigen::Vector2d> m_prevPts;

    std::vector<Eigen::Vector3d> m_odomPath;
    std::vector<Eigen::Vector3d> m_gaussianNewtonPath;

    tf::TransformListener m_tfListener;
    ros::Subscriber m_laserscanSub;
    ros::Publisher m_odomPub;
    ros::Publisher m_gaussianNewtonPub;
    ros::Publisher m_pcsPub;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "GaussianNewton_debug");

    GaussianNewtonDebug gn_debug;

    ros::spin();

    return (0);
}
