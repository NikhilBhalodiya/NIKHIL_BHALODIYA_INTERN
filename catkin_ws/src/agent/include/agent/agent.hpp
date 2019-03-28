#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <string>
#include <nav_msgs/Path.h>
#include <planner/get_plan.h>
#include <agent/update_goal.h>
#include "std_msgs/String.h"
#include <cstdlib>
#include "nav_msgs/OccupancyGrid.h"


namespace Multiagent_planner
{
    class Agent
    {
     public:
        Agent(const std::string &id_, const int x_, const int y_, const int yaw_):
        serial_id(id_),start_x(x_),start_y(y_),start_yaw(yaw_){}
        ~Agent(){}
//     private:
       std::string serial_id;
       int start_x;
       int start_y;
       int start_yaw;
       int goal_x;
       int goal_y;
       int goal_yaw;

       void setStart(const int &sx_, const int &sy_, const int &syaw_)
       {
           start_x = sx_;
           start_y = sy_;
           start_yaw = syaw_;
       }
       void setGoal(const int &gx_, const int &gy_, const int &gyaw_)
       {
           goal_x = gx_;
           goal_y = gy_;
           goal_yaw = gyaw_;
       }

    };

}

