#include <cmath>
#include <ros/ros.h>

namespace Multiagent_planner
{


    template <typename Object>
    struct Point
    {
        Point(){}
        Point(const Object &x_, const Object &y_) :
            x(x_),
            y(y_)
            {}
        ~Point() = default;
          bool operator==(const Point<Object> &rhs) const
          {
              return (rhs.x == x && rhs.y == y);
          }

          Object x;
          Object y;

    };


    struct GridNode
    {

        GridNode(){}
        ~GridNode() = default;
        int x;
        int y;
        int yaw;
        int id;
        GridNode *parent;
        int parent_id;

        int g_cost;
        int h_cost;
        int f_cost;

        //Update the node x y z values and ID which is uniques
        //For the case of 10x10 grid IDs varies from 0 to 99
        void init(const int &x_, const int &y_, const int &yaw_, const int &id_)
        {
            x = x_;
            y = y_;
            yaw = yaw_;
            id = id_;
        }
        //Set the current objects parent_id
        void set_parent(const int obj_id)
        {
            parent_id = obj_id ;
        }
        //Calculate f_cost and H_cost of the object
        void set_f_cost(GridNode &obj, GridNode &goalObj)
        {
            //euclidean distane between current position and goal
            obj.h_cost = std::sqrt((pow((goalObj.x - obj.x),2) + pow((goalObj.y - obj.y),2)));
            obj.f_cost = obj.g_cost + obj.h_cost;
        }

        bool operator==(const GridNode &rhs) const
        {
            return (x == rhs.x ) && (y == rhs.y);
        }
        //Comparisioin for getting minimum values from priority ques using piority_queue_name.top() function
        struct CheaperCost
        {

            bool operator()(const GridNode &lhs, const GridNode &rhs) const
            {
                return lhs.f_cost > rhs.f_cost;
            }
        };
    };



}




