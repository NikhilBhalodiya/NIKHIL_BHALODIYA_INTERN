#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <planner/get_plan.h>
#include <nav_msgs/Path.h>
#include <string>
#include <queue>
#include <gridnode.h>
#include "std_msgs/String.h"
#include <cmath>
#include <map>
#include <iterator>
#include <vector>
#include <algorithm>


namespace Multiagent_planner
{
    class Planner
    {
    public:
      Planner(ros::NodeHandle &nh, ros::NodeHandle &pnh);
     ~Planner();
    private:

      std::string agent_name;
      int agent_start_x;
      int agent_start_y;
      int agent_start_yaw;
      int agent_goal_x;
      int agent_goal_y;
      int agent_goal_yaw;
      std::string param_serial_id;
      int param_start_x,param_start_y,param_start_yaw;

      ros::ServiceServer get_plan_service;
      ros::Subscriber agent_current_pose_sub;

      geometry_msgs::PointStamped::ConstPtr current_pose;
      geometry_msgs::PoseStamped pose;
      nav_msgs::Path final_path;

      Multiagent_planner::GridNode start;
      Multiagent_planner::GridNode goal;
      Multiagent_planner::GridNode roadMap[100];
      Multiagent_planner::GridNode m_current_node;


      std::priority_queue<Multiagent_planner::GridNode, std::vector<Multiagent_planner::GridNode>, Multiagent_planner::GridNode::CheaperCost> frontier;
      std::priority_queue<Multiagent_planner::GridNode, std::vector<Multiagent_planner::GridNode>, Multiagent_planner::GridNode::CheaperCost> print_frontier;
      int neighbors_id[4];
      std::vector<int> path_id;
      std::vector<int> open;
      std::vector<int> close;

      void getParams(ros::NodeHandle &pnh);
      void agentfeedbackCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
      bool get_planCallback(planner::get_plan::Request &req, planner::get_plan::Response &res);
      bool A_star_planning();
      void initializeAll();
      void GenerateRoadmap(const int height, const int width);
      void getStartGoal();
      void printFrontier();
      bool checkIfItsGoal();
      void exploreCurrentNode();
      void getNeighbours();
      void backTracking();
      void generateNavMsg();

      int temp_node_id;
      int Motion[4][2] = {{1,0},
                          {0,1},
                          {-1,0},
                          {0,-1}};

    };

}
