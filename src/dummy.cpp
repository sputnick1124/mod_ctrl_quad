#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  std::cout << msg->pose.position.z << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("quad_dynamics/quad_pose", 1000, chatterCallback);

  ros::spin();

  return 0;
}
