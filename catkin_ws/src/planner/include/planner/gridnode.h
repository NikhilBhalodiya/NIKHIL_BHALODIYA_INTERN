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

        void init(const int &x_, const int &y_, const int &yaw_, const int &id_)
        {
            x = x_;
            y = y_;
            yaw = yaw_;
            id = id_;
        }
        void set_parent(const int obj_id)
        {
            parent_id = obj_id ;
        }

        void set_f_cost(GridNode &obj, GridNode &goalObj)
        {
            obj.h_cost = std::sqrt((pow((goalObj.x - obj.x),2) + pow((goalObj.y - obj.y),2)));
            obj.f_cost = obj.g_cost + obj.h_cost;
        }

        bool operator==(const GridNode &rhs) const
        {
            return (x == rhs.x ) && (y == rhs.y);
        }

        struct CheaperCost
        {

            bool operator()(const GridNode &lhs, const GridNode &rhs) const
            {
                return lhs.f_cost > rhs.f_cost;
            }
        };
    };



}




