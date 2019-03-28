#include <ros/ros.h>
#include <agent.hpp>
#include <AgentManager.hpp>


namespace Multiagent_planner
{

    AgentManager::AgentManager(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    {

         std::string ns;
         getParams(pnh);
         if(param_serial_id == "agent_1")
         {
            std::string ns = "sim1" ;
         }
         else if(param_serial_id == "agent_2")
         {
            std::string ns = "sim2" ;
         }
         else {
             ROS_ERROR("SERIAL_ID neighter \"agent_1\" nor \"agent_2\" first line");
         }

        agent_feedback_pub = nh.advertise<geometry_msgs::PointStamped>("/"+ns+"/agent_feedback", 100);
                                                                   ROS_ERROR("agent_feedback rostopic available");
        path_pub = nh.advertise<nav_msgs::Path>("/"+ns+"/path_project", 100);
                                                                   ROS_ERROR("Nav msg can be published now for frame /map_multiagent");
        update_goal_service = nh.advertiseService("/"+ns+"/update_goal", &AgentManager::update_goalCallback, this);
                                                                   ROS_ERROR("update_goal rosservice available");
        get_plan_client = nh.serviceClient<planner::get_plan>("/"+ns+"/get_plan");
                                                                   ROS_ERROR("get_plan client available to call get_plan rosservice");
        initializeAgents();

    }
    AgentManager::~AgentManager(){}

    void AgentManager::getParams(ros::NodeHandle &pnh)
    {
//        ROS_ERROR("%s", param_serial_id.c_str());
        pnh.getParam("/serial_id", param_serial_id);
        ROS_ERROR("%s", param_serial_id.c_str());
        pnh.getParam("/start_x", param_start_x);
        pnh.getParam("/start_y", param_start_y);
        pnh.getParam("/start_yaw", param_start_yaw);
    }

    void AgentManager::initializeAgents()
    {
        if(param_serial_id=="agent_1")
        {
             agent1 = new Multiagent_planner::Agent(param_serial_id,param_start_x,param_start_y,param_start_yaw);
                                                                     ROS_ERROR("agent_1 serial_id and start values assigned  %s %d %d",agent1->serial_id.c_str(),agent1->start_x,agent1->start_y);
        }
        else if(param_serial_id=="agent_2")
        {
            agent2 = new Multiagent_planner::Agent(param_serial_id,param_start_x,param_start_y,param_start_yaw);
                                                                     ROS_ERROR("agent_2 serial_id and start values assigned  %s %d %d",agent2->serial_id.c_str(),agent2->start_x,agent2->start_y);
        }
        else
        {
            ROS_ERROR("SERIAL_ID neighter \"agent_1\" nor \"agent_2\" ");
        }
     }

     void AgentManager::callGetPlanService(Agent &obj)
     {
         srv.request.serial_id = req_serial_id;
         srv.request.goal_x   = req_goal_x;
         srv.request.goal_y   = req_goal_y;
         srv.request.goal_yaw = req_goal_yaw;
                                                                      ROS_ERROR("get_plan srv request generated");
         if (get_plan_client.call(srv))
         {
                                                                      ROS_ERROR("srv call execution successfully");
             path_pub.publish(srv.response.path);
             obj.setStart(req_goal_x,req_goal_y,req_goal_yaw);
                                                                      ROS_ERROR("get_plan executed for %s",obj.serial_id.c_str());
         }
         else
         {
           ROS_ERROR("Failed to call service get_plan");
         }
      }

     bool AgentManager::update_goalCallback(agent::update_goal::Request &req, agent::update_goal::Response &res)
     {
                                                                       //ROS_ERROR("update_goal for given agent %s", req.serial_id.c_str());
         req_serial_id = req.serial_id;
         req_goal_x   = int(req.goal_x);
         req_goal_y   = int(req.goal_y);
         req_goal_yaw = int(req.goal_yaw);

         updateAgentInfo();
         return 1;
     }

     void AgentManager::updateAgentInfo()
     {
         if(req_serial_id == "agent_1")
         {
           agent1->setGoal(req_goal_x,req_goal_y,req_goal_yaw);
                                                                      //ROS_ERROR("goals for agent_1 updated");
           publishAgentFeedback(*agent1);
           callGetPlanService(*agent1);
         }
         else if(req_serial_id == "agent_2")
         {
           agent2->setGoal(req_goal_x,req_goal_y,req_goal_yaw);
                                                                       //ROS_ERROR("goals for agent_2 updated");
           publishAgentFeedback(*agent2);
           callGetPlanService(*agent2);
         }
     }

     void AgentManager::publishAgentFeedback(Agent &obj)
     {
         ROS_ERROR("agent serial_id and start values are  %s %d %d",obj.serial_id.c_str(),obj.start_x,obj.start_y);
         agent_current_pose.header.frame_id = req_serial_id;
         agent_current_pose.point.x = obj.start_x;
         agent_current_pose.point.y = obj.start_y;
         agent_current_pose.point.z = obj.start_yaw;
                                                                      //ROS_ERROR("start of %s sent to agent_feedback",obj.serial_id.c_str());
         agent_feedback_pub.publish(agent_current_pose);
     }

}
