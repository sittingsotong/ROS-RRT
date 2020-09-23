#include "rrt/rrt.h"
// helper function
int IndexOfPoint(const nav_msgs::MapMetaData& info, const geometry_msgs::Point& point){
    geometry_msgs::Point p = point;
    tf::Transform world_to_map;
    tf::poseMsgToTF(info.origin, world_to_map);
    world_to_map = world_to_map.inverse();
    tf::Point tf_pt;
    tf::pointMsgToTF(p, tf_pt);
    tf_pt = world_to_map*tf_pt;
    
    int grid_x = (int)((floor(tf_pt.x()) - info.origin.position.x));
    int grid_y = (int)((floor(tf_pt.y()) - info.origin.position.y));
    grid_y = info.height - grid_y; //index 1 is top left of map
    ROS_INFO("Grid values: %d, %d", grid_x, grid_y);

    int index = grid_x + grid_y * info.height;
    return index;
}

// class declaration
RRT::RRT(){
    // initialise origin node
    RRT::rrtNode newNode;
    newNode.pos = std::make_pair(0, 0);
    newNode.nodeID = 0;
    newNode.parentID = -1;

    rrtTree.push_back(newNode);

    //set other RRT parameters
    max_iterations = 30000;
    goal_radius = 1;
    step_size = 2;
}

RRT::RRT(nav_msgs::OccupancyGrid data, const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& end){
    map = data;
    map_width = data.info.width;
    map_height = data.info.height;

    //set other RRT parameters
    max_iterations = 100000;
    goal_radius = 1;
    step_size = 1;

    // initialise class variables for goal and origin point
    origin.first = start.pose.position.x;
    origin.second = start.pose.position.y;
    goal.first = end.pose.position.x;
    goal.second = end.pose.position.y;
    current_iteration = 0;

    // initialise origin node with parentID == -1
    RRT::rrtNode newNode;
    newNode.nodeID = 0;
    newNode.pos = origin;
    newNode.parentID = -1;
    rrtTree.push_back(newNode);

    ROS_INFO("Initialised");
}

std::pair<float, float> RRT::GetRandomPoint(){
    std::pair<float, float> point;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> x(0, map_width);
    std::uniform_real_distribution <> y(0, map_height);

    point = std::make_pair(x(gen), y(gen));
    ROS_INFO("Random point: %f, %f", point.first, point.second);
    return point;
}

float RRT::GetDistance(std::pair<float, float> start, std::pair<float, float> end){
    float x1 = start.first;
    float x2 = end.first;
    float y1 = start.second;
    float y2 = end.second;

    float distance = sqrt(pow((x1-x2), 2) + pow((y1-y2), 2));
    return distance;
}

int RRT::getClosestNeighbour(std::pair<float, float> point){
    int closest = 0;

    float closest_dist = std::numeric_limits<float>::infinity();
    float curr_dist = std::numeric_limits<float>::infinity();

    // iterate through all rrtNodes and find closest
    for(int i=0; i<rrtTree.size(); i++){
        curr_dist = GetDistance(point, rrtTree[i].pos);

        // check if distance between point and node is lesser than closest_dist
        if(curr_dist < closest_dist){
            closest_dist = curr_dist;
            closest = rrtTree[i].nodeID;
        }
    }
    ROS_DEBUG("distance: %f", closest_dist);
    // nodeID of node that is closest
    return closest;
}

bool RRT::IsValid(std::pair<float, float> start, std::pair<float, float> end){
    float start_x = start.first;
    float start_y = start.second;
    float end_x = end.first;
    float end_y = end.second;

    ROS_DEBUG("Getting angle");
    // angle between two points
    float theta = atan((end_y - start_y)/(end_x - start_x));

    // check if goal is valid first before entering loop
    // convert goal into a geometry_msgs/Point
    geometry_msgs::Point end_point;
    end_point.x = end_x;
    end_point.y = end_y;
    end_point.z = 0.0;

    // convert Point to its index and check if its occupied
    ROS_DEBUG("Converting to Index");
    int end_index = IndexOfPoint(map.info, end_point);

    if(map.data[end_index] > 65){ 
        ROS_DEBUG("Point is an obstacle");
        return false;
    }

    while(start_x < end_x && start_y < end_y){
        ROS_DEBUG("Start: %f, End: %f", start_x, start_y);
        // using step_size, walk down the path checking if each point is valid
        start_x += step_size * cos(theta);
        start_y += step_size * sin(theta);

        if(start_x >= end_x || start_y >= end_y){
            return true;
        }
        
        geometry_msgs::Point test_point;
        test_point.x = start_x;
        test_point.y = start_y;
        test_point.z = 0.0;

        int test_index = IndexOfPoint(map.info, test_point);
        int result = map.data[test_index];
        if(result > 65){
            ROS_INFO("Point is an obstacle");
            return false;
        }
    }
    return true;
}

bool RRT::ReachedGoal(int node){
    std::pair<float, float> curr_pos = rrtTree[node].pos;

    float dist = GetDistance(curr_pos, goal);
    ROS_DEBUG("distance to goal: %f", dist);
    if(dist < goal_radius){
        // check if path between these 2 have obstacle
        std::pair<float, float> point = rrtTree[node].pos;
        if(RRT::IsValid(point, goal)){
            current_iteration ++;
            RRT::rrtNode goalNode;
            goalNode.nodeID = current_iteration;
            goalNode.pos = goal;
            goalNode.parentID = node;

            rrtTree.push_back(goalNode);
            return true;
        }

        return false;
    } else {
        return false;
    }
}

int RRT::FindPath(){
    ROS_DEBUG("Finding Path");
    bool done = false;
    int goal_index = -1; 
    current_iteration = 0;

    // loop until goal is reached 
    while(!done && current_iteration < max_iterations){
        // step 1: find random point
        ROS_DEBUG("Getting random point");
        std::pair<float, float> ran_point = RRT::GetRandomPoint();

        ROS_DEBUG("Getting closest node");
        // step 2: get closest node to point
        int closest_node = RRT::getClosestNeighbour(ran_point);
        ROS_DEBUG("Closest Node: %d", closest_node);

        ROS_INFO("Checking if valid");
        // step 3: check if valid point
        std::pair<float, float> point = rrtTree[closest_node].pos; 
        if(RRT::IsValid(point, ran_point)){
            ROS_INFO("Is Valid");
            current_iteration ++;
            RRT::rrtNode ranNode;
            ranNode.nodeID = current_iteration;
            ranNode.pos = ran_point;
            ranNode.parentID = closest_node;
            rrtTree.push_back(ranNode);

            // current_iteration should equal new nodeID?
            // check if goal is within goal_radius;
            ROS_DEBUG("Checking if reached goal");
            if (ReachedGoal(current_iteration)){
                goal_index = current_iteration;
                done = true;
            }
        }

        if(current_iteration == max_iterations){
            ROS_INFO("Max iteration reached!");
            return -1;
        }
    }

    return goal_index;
}

std::vector<geometry_msgs::PoseStamped> RRT::BuildPath(int goal_index, geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal){
    // vector of PoseStamped that creates the path
    std::vector<geometry_msgs::PoseStamped> path;

    if(goal_index == -1){
        return path;
    }
    
    // find the path from the back
    std::list<int> index_list;
    index_list.push_front(goal_index);
    int parent_index = rrtTree[goal_index].parentID;
    while(parent_index > 0){
        index_list.push_front(parent_index);
        parent_index = rrtTree[parent_index].parentID;
    }
    index_list.push_front(0); 

    // add the path from the front as PoseStamped
    for(int i=0; i<index_list.size(); i++){
        ros::Time current_time = ros::Time::now();
        if(i == 0){
            start.header.stamp = current_time;
            path.push_back(start);
        }
        geometry_msgs::PoseStamped point;
        point.pose.position.x = rrtTree[i].pos.first;
        point.pose.position.y = rrtTree[i].pos.second;
        point.pose.position.z = 0.0;

        point.pose.orientation = tf::createQuaternionMsgFromYaw(0);

        point.header.frame_id="map";
        point.header.stamp = current_time;
        
        path.push_back(point);
    }

    ros::Time current_time = ros::Time::now();
    goal.header.stamp = current_time;
    path.push_back(goal);

    return path;
}

std::vector<geometry_msgs::PoseStamped> RRT::RerunAlgo(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal){
    std::vector<geometry_msgs::PoseStamped> path;

    for(int i=0; i<rrtTree.size(); i++){
        ros::Time current_time = ros::Time::now();
        if(i == 0){
            start.header.stamp = current_time;
            start.pose.position.x *= map.info.resolution;
            start.pose.position.y *= map.info.resolution;
            path.push_back(start);
        }

        geometry_msgs::PoseStamped point;
        point.pose.position.x = rrtTree[i].pos.first * map.info.resolution;
        point.pose.position.y = rrtTree[i].pos.second * map.info.resolution;
        point.pose.position.z = 0.0;

        point.pose.orientation = tf::createQuaternionMsgFromYaw(0);

        point.header.frame_id="map";
        point.header.stamp = current_time;
        
        path.push_back(point);
    } 

    ros::Time current_time = ros::Time::now();
    goal.header.stamp = current_time;
    goal.pose.position.x *= map.info.resolution;
    goal.pose.position.y += map.info.resolution;
    path.push_back(goal);    
    
    return path;
}
