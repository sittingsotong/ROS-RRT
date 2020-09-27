#ifndef graph_h
#define graph_h

#include <cstdlib>
#include <vector>
#include <utility>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"


// graph is an array of adjancency lists 
class Graph{
    private:
        // vertex node
        struct Vertex{
            int vertex_id;
            std::pair<float, float> pos;
            Vertex* next;
        };

        // adjacency list for each vertex start
        struct adjList {
            Vertex *head;
        };

        // total number of vertex
        int V;
        // graph is an aray of adjacency lists
        std::vector<Graph::adjList> arr;

    public:
        // constructor with starting point
        Graph(const geometry_msgs::PoseStamped& start);

        // adds new vertex point
        void addVertex(int parent_id, int new_id, std::pair<float, float> point);

        void printGraph();
};

#endif