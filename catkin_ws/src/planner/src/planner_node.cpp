#include "ros/ros.h"
#include "planner.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner_node");
  {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  Multiagent_planner::Planner planner_node(nh,pnh);

  while(ros::ok)
  {
      ros::spin();
  }
  return 0;
  }
}
