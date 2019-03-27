#include "ros/ros.h"
#include <AgentManager.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agent_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    Multiagent_planner::AgentManager AgentManager(nh, pnh);

    while(ros::ok)
    {
      ros::spin();
    }
    return 0;
}
