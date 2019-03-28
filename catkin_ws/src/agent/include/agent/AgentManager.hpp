#include <ros/ros.h>
#include <agent.hpp>


namespace Multiagent_planner
{
    class AgentManager
    {
        public:
            AgentManager(ros::NodeHandle &nh, ros::NodeHandle &pnh);
            ~AgentManager();

//        private:
            //taking values from parameter server
            int param_start_x,param_start_y,param_start_yaw;
            std::string param_serial_id;
            //taking value when update goal request generates
            int req_goal_x, req_goal_y,req_goal_yaw;
            std::string req_serial_id;

            //flags running some loops only once
            bool callback_flag=0;
            bool params_needed_flag=1;

            //publisher subscriber service client definations
            ros::Publisher agent_feedback_pub;
            ros::Publisher path_pub;
            ros::ServiceServer  update_goal_service;
            ros::ServiceClient get_plan_client;

            //get_plan service client variable
            planner::get_plan srv;

            //two pointers to the Agent class objects
            Multiagent_planner::Agent *agent1;
            Multiagent_planner::Agent *agent2;

            geometry_msgs::PointStamped agent_current_pose;
            nav_msgs::OccupancyGrid blank_grid;

            //getting parameters
            void getParams(ros::NodeHandle &pnh);
            //creating respective Agent objects as per the id
            void initializeAgents();
            //call get plan service client to send goal and id
            void callGetPlanService(Agent &obj);
            //update_goal service callback function
            bool update_goalCallback(agent::update_goal::Request &req, agent::update_goal::Response &res);
            //update respective agent class object info
            void updateAgentInfo();
            //publish agent's start and id to the agent_feedback topic
            void publishAgentFeedback(Agent &obj);

      };

}


//std::int8_t map_data[100];

//            blank_grid.header.frame_id = "/map_multiagent";
//            blank_grid.info.width = 10;
//            blank_grid.info.height = 10;
//            for(int i=0; i<100; i++)
//            {
//                 map_data[i] = 0;
//            blank_grid.data.push_back(map_data[i]);
//            }
//            map_pub.publish(blank_grid);
