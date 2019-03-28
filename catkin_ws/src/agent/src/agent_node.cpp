#include "ros/ros.h"
#include <AgentManager.hpp>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "agent_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string s;
    pnh.getParam("/serial_id",s);
    ROS_ERROR("ssssssssssssssssssssssssssssssssssssssss node node %s",s.c_str());

    Multiagent_planner::AgentManager AgentManager(nh, pnh);

    while(ros::ok)
    {
      ros::spin();
    }
    return 0;
}
