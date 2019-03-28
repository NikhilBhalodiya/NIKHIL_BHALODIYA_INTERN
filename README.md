# NIKHIL_BHALODIYA_INTERN
Multi_agent_planner


l>>>>>>>>>>Open terminal one                                                                                                     
$cd NIKHIL_BHALODIYA_INTERN/catkin_ws                                                                                                   
$catkin_make                                                                                                                             
$roslaunch agent agent_node.launch serial_id:=agent_1 start_x:=0 start_y:=0 start_yaw:=0                                               

l>>>>>>>>>>>>>>>>open terminal 2                                                                                           
$roslaunch agent agent_node.launch serial_id:=agent_2 start_x:=5 start_y:=5 start_yaw:=0                             

l>>>>>>>>>>>>>>>open terminal three                                                                              
$rosservice list /update_goal agent_2 10 8 0                                                                     
// you can see the output on Rviz and check the ROS_INFO on terminal two. . . .                                  
(Please consider ROS_ERROR msgs as ROS_INFO msgs somehow I can't see the ROS_INFO msgs on terminal )
$rosservice list /update_goal agent_2 1 1 0 

Ideally but its not working >>>>>>>>>>> $rosservice list /update_goal agent_1 5 5 0 



Description of the architecture
catkin_ws has two pkgs
Planner and agnet 

agent pkg creates agent_node
planner pkg creates planner_node

roslaunch launches both nodes as well as it launch both in different namespaces named sim1 and sim2
agent_node creates an object of AgentManeger class which further creates two objects of the class Agent.

planner node creates an object of the class Planner which handles A_star planning and send the response back as te nav_msgs/Path data type





TEST CASES SUCCESSFULLY works are

l>>>>>>>>>>>>>>>>open terminal 1                                                                                             
$cd NIKHIL_BHALODIYA_INTERN/catkin_ws     
$roslaunch agent agent_node.launch serial_id:=agent_1 start_x:=5 start_y:=5 start_yaw:=0                                   
l>>>>>>>>>>>>>>>open terminal 2                                                                                           
$rosservice list /update_goal agent_1 9 9 0    

Whiever is agent is launched last is available to plan the path 
    launch a2 5 5 0;  launch a1 5 5 0; update goal a1 9 9 0 also workiing 


//ctrl + c the terminal 1                                          

l>>>>>>>>>>>>>>>>open terminal 1           
$cd NIKHIL_BHALODIYA_INTERN/catkin_ws     
$roslaunch agent agent_node.launch serial_id:=agent_2 start_x:=5 start_y:=5 start_yaw:=0                                     
l>>>>>>>>>>>>>>>open terminal 2                                                                                               
$rosservice list /update_goal agent_1 9 9 0                                                                                                                                                                                              
ctrl + C terminal 2


