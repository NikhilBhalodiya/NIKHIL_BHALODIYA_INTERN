#include <ros/ros.h>
#include <planner.hpp>



namespace Multiagent_planner
{
    Planner::Planner(ros::NodeHandle &nh, ros::NodeHandle &pnh)
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
        get_plan_service = nh.advertiseService("/"+ns+"/get_plan", &Planner::get_planCallback, this);                ROS_ERROR("/get_plan service available");
        agent_current_pose_sub = nh.subscribe("/"+ns+"/agent_feedback", 100, &Planner::agentfeedbackCallback, this); ROS_ERROR("/agent_feedback is subscribed");
    }
    Planner::~Planner(){}

    void Planner::getParams(ros::NodeHandle &pnh)
    {
        ROS_ERROR("%s", param_serial_id.c_str());
        pnh.getParam("/serial_id", param_serial_id);
        ROS_ERROR("%s", param_serial_id.c_str());
        pnh.getParam("/start_x", param_start_x);
        pnh.getParam("/start_y", param_start_y);
        pnh.getParam("/start_yaw", param_start_yaw);
    }

    void Planner::agentfeedbackCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
       current_pose = msg;
       agent_name = current_pose->header.frame_id;        ROS_ERROR("%s start info available to Planner Node ", agent_name.c_str());
       agent_start_x = current_pose->point.x;
       agent_start_y = current_pose->point.y;
       agent_start_yaw = current_pose->point.z;
    }

    bool Planner::get_planCallback(planner::get_plan::Request  &req, planner::get_plan::Response &res)
    {
        agent_goal_x = req.goal_x;                         ROS_ERROR("Get Plan service is called");
        agent_goal_y = req.goal_y;
        agent_goal_yaw = req.goal_yaw;
                                                      //ROS_ERROR("Started Path Planning");
                                                      //ROS_ERROR("For agent name %s", req.serial_id.c_str());
                                                      //ROS_ERROR("start_x %d, start_y %d ",agent_start_x,agent_start_y);
                                                      //ROS_ERROR("goal_x %d, goal_y %d ",agent_goal_x,agent_goal_y);
        A_star_planning();
        res.path = final_path;
        return 1;
     }

    bool Planner::A_star_planning()
    {
        initializeAll();

        GenerateRoadmap(10,10);                            ROS_ERROR("Road Map Generated");
        getStartGoal();

        frontier.push(roadMap[start.id]);
        open.push_back(start.id);

        while(!open.empty())
        {
//         printFrontier();
           m_current_node = frontier.top();
                                                            // ROS_ERROR("current Node id is %d and his f_cost is %d ",m_current_node.id,roadMap[m_current_node.id].f_cost);
           if(checkIfItsGoal())
           {
               return 1;
           }

           const std::vector<int>::iterator &it_int = std::find (open.begin(), open.end(), m_current_node.id);

           bool print_open_set_flag = false;
           if(print_open_set_flag)
           {
               if (it_int != open.end())
               {
                   std::cout << "Element " << m_current_node.id <<" found at position : " ;
                   std:: cout << it_int - open.begin() + 1 << "\n" ;
               }
           }

           frontier.pop();
           open.erase(it_int);
           close.push_back(m_current_node.id);             //ROS_ERROR("node %d is added to closed set",m_current_node.id);
           exploreCurrentNode();
         }
         return 1;
     }

    void Planner::initializeAll()
    {
        while(!frontier.empty())
        {
        frontier.pop();
        }
        open.clear();
        close.clear();
        path_id.clear();
        final_path.poses.clear();
        final_path.header.frame_id.clear();
        for(int i = 0;i<100;i++)
        {
            roadMap[i].g_cost = 0;
            roadMap[i].h_cost = 0;
            roadMap[i].f_cost = 0;
            roadMap[i].set_parent(0);
        }
    }

    void Planner::GenerateRoadmap(const int height, const int width)
    {
        for (int i=0;i<(height*width);i++)
        {
          roadMap[i].init(int(i/width), int(i/height), 0,i);
        }
    }

    void Planner::getStartGoal()
    {

        start.init(agent_start_x,agent_start_y,agent_start_yaw,(agent_start_x*10) + agent_start_y);
        goal.init(agent_goal_x,agent_goal_y,agent_goal_yaw,(agent_goal_x*10) + agent_goal_y);

        start.g_cost = 0;
        start.set_parent(start.id);
        roadMap[start.id].parent_id = start.id;

        goal.g_cost = 9999;
        goal.set_f_cost(goal,goal);
        roadMap[goal.id].set_f_cost(roadMap[start.id],goal);

        start.set_f_cost(start,goal);
        roadMap[start.id].set_f_cost(roadMap[start.id],goal);
                                                           ROS_ERROR("Start is at %d and f_cost %d Goal_id is %d ", start.id, start.f_cost, goal.id);
    }

    void Planner::printFrontier()
    {
        print_frontier =  frontier;
        while (!print_frontier.empty())
        {
            ROS_ERROR("frontier element id %d  and its f_cost %d", print_frontier.top().id,roadMap[print_frontier.top().id].f_cost);
            print_frontier.pop();
        }
    }

    bool Planner::checkIfItsGoal()
    {
        if(m_current_node.id == goal.id)
        {
                                                             ROS_ERROR("GOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOAL");
            roadMap[goal.id].parent_id = roadMap[m_current_node.id].parent_id;
            backTracking();
            generateNavMsg();
            return 1;                                        ROS_ERROR("Path generation Complete");
        }

        return 0;
      }

    void Planner::exploreCurrentNode()
    {
        getNeighbours();
        for (int neighbor = 0; neighbor<4; neighbor++)
        {

           const std::vector<int>::iterator &it_close = std::find(close.begin(), close.end(),neighbors_id[neighbor]);
           if(it_close != close.end())
           {
             continue;
           }

            int tentative_gScore = roadMap[m_current_node.id].g_cost + 1;

            const std::vector<int>::iterator &it_op = std::find(open.begin(), open.end(), neighbors_id[neighbor]);
            if(it_op == open.end())
            {
               frontier.push(roadMap[neighbors_id[neighbor]]);
               open.push_back(neighbors_id[neighbor]);
            }
            else if(tentative_gScore >= roadMap[neighbors_id[neighbor]].g_cost)
            {
               continue;
            }
             roadMap[neighbors_id[neighbor]].set_parent(m_current_node.id);
             roadMap[neighbors_id[neighbor]].g_cost = tentative_gScore;
             roadMap[neighbors_id[neighbor]].set_f_cost(roadMap[neighbors_id[neighbor]],goal);
        }
    }

    void Planner::getNeighbours()
    {

        for (int i =0; i <4;i++)
        {
           int x = m_current_node.x+Motion[i][0];
           int y = m_current_node.y+Motion[i][1];

           if(x>= 10 || x<0 || y>= 10 || y<0)
           {
              continue;
           }
           int node_id = x*10 + y;
           roadMap[node_id].init(x,y,0,node_id);
           neighbors_id[i] = node_id;

        }
    }

    void Planner::backTracking()
    {
        temp_node_id = roadMap[goal.id].parent_id;
        path_id.push_back(goal.id);
                                                            ROS_ERROR("Parent_id of Node %d is %d", goal.id, temp_node_id );
        while(temp_node_id != roadMap[temp_node_id].parent_id)
        {                                                   ROS_ERROR("Parent_id of Node %d is %d", temp_node_id, roadMap[temp_node_id].parent_id );
             path_id.push_back(temp_node_id);
             temp_node_id = roadMap[temp_node_id].parent_id;
        }
        path_id.push_back(temp_node_id);

    }

    void Planner::generateNavMsg()
    {

        final_path.header.frame_id = "/map_multiagent";      ROS_ERROR("Nav msg generated for Frame /map_multiagent");

        for (int i = 0; i < path_id.size(); i++)
        {
            pose.pose.position.x = (roadMap[path_id.at(i)].x) - 5;
            pose.pose.position.y = (roadMap[path_id.at(i)].y) - 5;
            final_path.poses.push_back(pose);
        }
                                                             ROS_ERROR("final Pose of agent in Nav msg is %f %f", final_path.poses[0].pose.position.x + 5 ,final_path.poses[0].pose.position.y + 5 );
    }
}

