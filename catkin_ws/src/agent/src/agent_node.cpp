#include "ros/ros.h"
#include <AgentManager.hpp>


int main(int argc, char **argv)
{
    //Agent_node created which creates and object of AgentManager class and AgentManager handles 2 Agent class objects with repective namespace
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
