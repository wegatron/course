#include <ros/ros.h>
#include <std_msgs/String.h>

void get_msg_callback(const std_msgs::String::ConstPtr &msg)
{
  ROS_INFO("get %s", msg->data.c_str());
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "hello_get");

  ros::NodeHandle nh("~");

  ros::Subscriber sub_image = nh.subscribe("/chatter", 1000, get_msg_callback);

  ros::spin();
  return 0;
}
