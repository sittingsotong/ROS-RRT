#ifndef rrt_h
#define rrt_h

#include "ros/ros.h"
#include <vector>
#include <utility>
#include <cstdlib>
#include <random>
#include <cmath>
#include <limits>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"

class RRT{
    private:
        int max_iterations;
        int current_iteration;
        float goal_radius;
        float step_size;
        std::pair<float, float> origin;
        std::pair<float, float> goal;
        nav_msgs::OccupancyGrid map;
        int map_width;
        int map_height;

        // struct to represent each node
        struct rrtNode{
            int nodeID;
            std::pair<float, float> pos;
            int parentID;
            RRT::rrtNode* next;
        };

        // tree of RRT paths
        std::vector<RRT::rrtNode> rrtTree;

        // vector of pointers to nodes (tree structure)
        std::vector<RRT::rrtNode*> graph;

    public:
        // default constructor
        RRT();

        // constructor with OccupancyGrid map and start and goal points
        RRT(const nav_msgs::OccupancyGrid& data, const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& end);

        // generates random point
        std::pair<float, float> GetRandomPoint();

        // get a distance value between 2 points
        float GetDistance(std::pair<float, float> start, std::pair<float, float> end);

        // finds the nearest neighbour
        int getClosestNeighbour(std::pair<float, float> point);

        // check if the path is valid between 2 points
        bool IsValid(std::pair<float, float> start, std::pair<float, float> end);

        // check if node is within goal_radius and theres a clear path
        bool ReachedGoal(int node);

        // call this function to run RRT algo
        // returns nodeID of node right before goal
        int FindPath();

        // builds the plan after the path has been found 
        // returns a vector of PoseStamped to be added to Path message
        std::vector<geometry_msgs::PoseStamped> BuildPath(int goal_index, geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal);

        // methods for graph
        // constructor with starting point
        void createGraph(RRT::rrtNode node);

        // adds new vertex point
        void addNode(int parent_id, RRT::rrtNode node);

        void printGraph();
};

#endif