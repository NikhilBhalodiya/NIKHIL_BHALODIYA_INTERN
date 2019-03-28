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
      std::string param_serial_id;

      //paramters for planning
      std::string agent_name;
      int agent_start_x;
      int agent_start_y;
      int agent_start_yaw;
      int agent_goal_x;
      int agent_goal_y;
      int agent_goal_yaw;

      //get_plan service and agent_feedback subscriber
      ros::ServiceServer get_plan_service;
      ros::Subscriber agent_current_pose_sub;

      //taking current pose from agent_feedback topic
      geometry_msgs::PointStamped::ConstPtr current_pose;

      //genrating path to be sent as response to get_plan service which will be published to rviz
      geometry_msgs::PoseStamped pose;
      nav_msgs::Path final_path;

      Multiagent_planner::GridNode start;
      Multiagent_planner::GridNode goal;

      //Generating an array of 100 objects of GridNode class (Node Id of the object will be exact same as position of the object in the array)
      Multiagent_planner::GridNode roadMap[100];
      //Node to be expolored found from minimum priority queue value
      Multiagent_planner::GridNode current_node;

      //creating two priority queue one for getting minimum f_cost objets and once for print the current priority queue
      std::priority_queue<Multiagent_planner::GridNode, std::vector<Multiagent_planner::GridNode>, Multiagent_planner::GridNode::CheaperCost> frontier;
      std::priority_queue<Multiagent_planner::GridNode, std::vector<Multiagent_planner::GridNode>, Multiagent_planner::GridNode::CheaperCost> print_frontier;
      //vector of Open_set node IDs
      std::vector<int> open;
      //vector of ClosedS_set node IDs
      std::vector<int> close;

      //array of neighbour IDs of node to be explores
      int neighbors_id[4];
      //vector of Ids of nodes to reach from start to goal
      std::vector<int> path_id;


      void agentfeedbackCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
      bool getplanCallback(planner::get_plan::Request &req, planner::get_plan::Response &res);
      bool A_star_planning();
      //Clear all the values such as Priority queues, vectors, Roadmap array
      void initializeAll();
      void GenerateRoadmap(const int height, const int width);
      //getting start and goal from the agent_feedback sent msg
      void updateStartGoal();
      //printing current priotiry queue frintier
      void printFrontier();
      bool checkIfItsGoal();
      void exploreCurrentNode();
      void getNeighbours();
      void backTracking();
      //Generating the Nav msg inorder to send as response to get_pla which will be published to Rviz
      void generateNavMsg();

      //used for backtracking
      int temp_node_id;
      //to get the 4 connected nodes of current nodes
      int Motion[4][2] = {{1,0},
                          {0,1},
                          {-1,0},
                          {0,-1}};

    };

}
