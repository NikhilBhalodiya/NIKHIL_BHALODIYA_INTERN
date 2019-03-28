#include <ros/ros.h>
#include <planner.hpp>

namespace Multiagent_planner
{
    //Constructor
    Planner::Planner(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    {
         std::string ns;
         pnh.getParam("/serial_id", param_serial_id);
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
        get_plan_service = nh.advertiseService("/"+ns+"/get_plan", &Planner::getplanCallback, this);                ROS_ERROR("/get_plan service available");
        agent_current_pose_sub = nh.subscribe("/"+ns+"/agent_feedback", 100, &Planner::agentfeedbackCallback, this); ROS_ERROR("/agent_feedback is subscribed");
    }
    Planner::~Planner(){}

    //Taking current position of the agent from the agent_feedback topic
    void Planner::agentfeedbackCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
       current_pose = msg;
       agent_name = current_pose->header.frame_id;        ROS_ERROR("%s start info available to Planner Node ", agent_name.c_str());
       agent_start_x = current_pose->point.x;
       agent_start_y = current_pose->point.y;
       agent_start_yaw = current_pose->point.z;
    }

    bool Planner::getplanCallback(planner::get_plan::Request  &req, planner::get_plan::Response &res)
    {
        agent_goal_x = req.goal_x;                           ROS_ERROR("Get Plan service is called");
        agent_goal_y = req.goal_y;
        agent_goal_yaw = req.goal_yaw;
                                                            //ROS_ERROR("Started Path Planning");
                                                            //ROS_ERROR("For agent name %s", req.serial_id.c_str());
                                                            //ROS_ERROR("st_x %d sty %d",agent_start_x,agent_start_y);
                                                            //ROS_ERROR("goal_x %d goal_y %d",agent_goal_x,agent_goal_y);
        //Start Planning
        A_star_planning();
        //sending the response of Nav_msg/Path types
        res.path = final_path;
        return 1;
     }

    bool Planner::A_star_planning()
    {
        //clear all  priority_queue frontier, vectors open close path_id final_path,  array roadmap
        initializeAll();

        //generating 10x 10 grid having 100 nodes with unique IDs variers from 0 to 99 in this case
        //Id of the object located at roadmap array index 0 will be zero, at index 1 will be 1  .. . . .. .at index 99 will 99.
        GenerateRoadmap(10,10);                                   ROS_ERROR("Road Map Generated");
        //getting current position
        updateStartGoal();
        //adding current node object to priority queue frontier
        frontier.push(roadMap[start.id]);
        //adding the current node ID to open vector
        open.push_back(start.id);

        //untill open is not empty
        while(!open.empty())
        {
//         printFrontier();  //uncomment it if print priotity que
           //current node is the Node in the priority queue with the lowest F_cost value; where F_coat = g_cost + h_cost
           current_node = frontier.top();
                                                            // ROS_ERROR("current Node id is %d and his f_cost is %d ",current_node.id,roadMap[current_node.id].f_cost);
           //If current node objet is goal object break the while
           if(checkIfItsGoal())
           {
               return 1;
           }

           //Find the iterator for the current node Id in vector open
           const std::vector<int>::iterator &it_int = std::find (open.begin(), open.end(), current_node.id);

           bool print_open_set_flag = false;   //make this true if open vector needs to be pront
           if(print_open_set_flag)
           {
               if (it_int != open.end())
               {
                   ROS_ERROR("Element %d found at positon %d",current_node.id,(it_int - open.begin() + 1));

               }
           }
           //Remove the object at top(with minimum F_cost value) from the priority Queue as it will explored once
           frontier.pop();
           //erase it's ID from the open vector as well
           open.erase(it_int);
           //add the ID of the current node object to close vector
           close.push_back(current_node.id);             //ROS_ERROR("node %d is added to closed set",current_node.id);
           exploreCurrentNode();
         }
         return 1;
     }

    void Planner::initializeAll()
    {
        while(!frontier.empty())
        {
         ROS_ERROR("deleting.....");
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

    void Planner::updateStartGoal()
    {
        //uodating the start and goal position to the respective node object
        start.init(agent_start_x,agent_start_y,agent_start_yaw,(agent_start_x*10) + agent_start_y);
        goal.init(agent_goal_x,agent_goal_y,agent_goal_yaw,(agent_goal_x*10) + agent_goal_y);
       //initially g_cost of start is zer0
        start.g_cost = 0;
        //parent of the start is start itself
        start.set_parent(start.id);
        //placing start object at Roadmap's startId corresponding Index
        roadMap[start.id].parent_id = start.id;

        //initially g_cost of goal is infinite
        goal.g_cost = 9999;
        goal.set_f_cost(goal,goal);
        //placing goal object at Roadmap's goalId corresponding Index
        roadMap[goal.id].set_f_cost(roadMap[start.id],goal);

        //calculating start's F_cost
        start.set_f_cost(start,goal);
        roadMap[start.id].set_f_cost(roadMap[start.id],goal);
                                                           ROS_ERROR("Start is at %d and f_cost %d Goal_id is %d ", start.id, start.f_cost, goal.id);
    }
    //printing priotity queue with element id and its F_cost
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
        if(current_node.id == goal.id)
        {
                                                             ROS_ERROR("GOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOAL");
            roadMap[goal.id].parent_id = roadMap[current_node.id].parent_id;
            backTracking();
            generateNavMsg();
            return 1;                                        ROS_ERROR("Path generation Complete");
        }

        return 0;
      }

    void Planner::exploreCurrentNode()
    {
        //generating and array of four connected neighbour of the current_node
        getNeighbours();

        for (int neighbor = 0; neighbor<4; neighbor++)
        {
           //for each neighbour check if its in the close vecotr already
           const std::vector<int>::iterator &it_close = std::find(close.begin(), close.end(),neighbors_id[neighbor]);
           if(it_close != close.end())
           {
             continue;
           }

            ////for each neighbour check if its not in the open vecotr
            const std::vector<int>::iterator &it_op = std::find(open.begin(), open.end(), neighbors_id[neighbor]);
            if(it_op == open.end())
            {
               //if its not in open vector then adding it to priority queue frontier as well as open vector
               frontier.push(roadMap[neighbors_id[neighbor]]);
               open.push_back(neighbors_id[neighbor]);
            }
             //defining its parent_Id, g_cost and F_cost
             roadMap[neighbors_id[neighbor]].set_parent(current_node.id);
             roadMap[neighbors_id[neighbor]].g_cost = roadMap[current_node.id].g_cost + 1;;
             roadMap[neighbors_id[neighbor]].set_f_cost(roadMap[neighbors_id[neighbor]],goal);
        }
    }

    void Planner::getNeighbours()
    {
        //check for four connected neighbour
        for (int i =0; i <4;i++)
        {
           int x = current_node.x+Motion[i][0];
           int y = current_node.y+Motion[i][1];
           //if its out of grid here grid is 10x 10
           if(x>= 10 || x<0 || y>= 10 || y<0)
           {
              continue;
           }
           //calculate node id for each neighbour
           int node_id = x*10 + y;
           //update the node object info at the nodeID index of roadmap Array
           roadMap[node_id].init(x,y,0,node_id);
           neighbors_id[i] = node_id;

        }
    }

    void Planner::backTracking()
    {
        temp_node_id = roadMap[goal.id].parent_id;
        path_id.push_back(goal.id);
        //untill the node has node_id and parent_id same                                                     ROS_ERROR("Parent_id of Node %d is %d", goal.id, temp_node_id );
        while(temp_node_id != roadMap[temp_node_id].parent_id)
        {                                                   ROS_ERROR("Parent_id of Node %d is %d", temp_node_id, roadMap[temp_node_id].parent_id );
             path_id.push_back(temp_node_id);
             temp_node_id = roadMap[temp_node_id].parent_id;
        }
        path_id.push_back(temp_node_id);

    }

    void Planner::generateNavMsg()
    {
        //generated msg is w.r.t to frmae /map_multiagent
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

