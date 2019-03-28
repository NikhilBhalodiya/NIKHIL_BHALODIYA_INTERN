#include <ros/ros.h>
#include <agent.hpp>


namespace Multiagent_planner
{
    class AgentManager
    {
        public:
            AgentManager(ros::NodeHandle &nh, ros::NodeHandle &pnh);
            ~AgentManager();

        private:
            int param_start_x,param_start_y,param_start_yaw;
            int req_goal_x, req_goal_y,req_goal_yaw;
            bool callback_flag=0;
            bool params_needed_flag=1;
            std::string req_serial_id;
            std::string param_serial_id;

            ros::Publisher agent_feedback_pub;
            ros::Publisher path_pub;
            ros::ServiceServer  update_goal_service;
            ros::ServiceClient get_plan_client;

            planner::get_plan srv;
            Multiagent_planner::Agent *agent1;
            Multiagent_planner::Agent *agent2;

            geometry_msgs::PointStamped agent_current_pose;

            void getParams(ros::NodeHandle &pnh);
            void initializeAgents();
            void callGetPlanService(Agent &obj);
            bool update_goalCallback(agent::update_goal::Request &req, agent::update_goal::Response &res);
            void updateAgentInfo();
            void publishAgentFeedback(Agent &obj);

      };

}
