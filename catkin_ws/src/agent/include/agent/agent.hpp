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


namespace Multiagent_planner
{
    class Agent
    {
     public:
        Agent(const std::string &a, const int b, const int c, const int d):
        serial_id(a),start_x(b),start_y(c),start_yaw(d){}
        ~Agent(){}

       std::string serial_id;
       int start_x;
       int start_y;
       int start_yaw;
       int goal_x;
       int goal_y;
       int goal_yaw;

       void setStart(const int &x_, const int &y_, const int &yaw_)
       {
           start_x = x_;
           start_y = y_;
           start_yaw = yaw_;
       }
       void setGoal(const int &x_, const int &y_, const int &yaw_)
       {
           goal_x = x_;
           goal_y = y_;
           goal_yaw = yaw_;
       }

       ros::Publisher agent_feedback_pub;
       ros::ServiceServer update_goal_service;
       geometry_msgs::PointStamped agent_current_pose;
       nav_msgs::Path final_path1;

       bool update_goalCallback(agent::update_goal::Request &req, agent::update_goal::Response &res);

      };

}

