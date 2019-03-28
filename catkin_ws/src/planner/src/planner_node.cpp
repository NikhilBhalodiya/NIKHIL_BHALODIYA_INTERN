#include "ros/ros.h"
#include "planner.hpp"

int main(int argc, char **argv)
{
  //Generating Object_node with respective namespace
  ros::init(argc, argv, "planner_node");
  {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  //Creating an object of Planner Class
  Multiagent_planner::Planner planner(nh,pnh);

  while(ros::ok)
  {
      ros::spin();
  }
  return 0;
  }
}
