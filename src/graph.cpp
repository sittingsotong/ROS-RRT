#include "rrt/graph.h"

Graph::Graph(const geometry_msgs::PoseStamped& start){
    V = 1;
    Graph::Vertex* start_vertex; 
    start_vertex->vertex_id = 0;
    start_vertex->pos = std::make_pair(start.pose.position.x, start.pose.position.y);
    start_vertex->next =NULL;

    Graph::adjList list;
    list.head = start_vertex;
    arr.push_back(list);
}

void Graph::addVertex(int parent_id, int new_id, std::pair<float, float> point){
    V++;
    Graph::Vertex* new_vertex;
    new_vertex->vertex_id = new_id;
    new_vertex->pos = point;
    new_vertex->next = arr[parent_id].head;
    arr[parent_id].head = new_vertex;
}