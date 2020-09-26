#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include <vector>

#include "rrt/rrt.h"

// global variables 
nav_msgs::OccupancyGrid mapData;
geometry_msgs::PoseStamped start, goal;
bool goal_received = false;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  mapData = *msg;

  ROS_INFO("Got map %d, %d", mapData.info.width, mapData.info.height);
}

// same callback for the start and end coordinateS?
void startCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  start = *msg;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  goal = *msg;
  goal_received = true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "planner");

  ros::NodeHandle n;

  ros::Rate r(10);
  // subscribe to map topic 
  ros::Subscriber map_sub = n.subscribe("map", 1000, mapCallback);

  // subscribe to coordinates topic (start and end coordinates)
  // get coordinates of type nav_msgs::PoseStamped 
  ros::Subscriber start_sub = n.subscribe("start", 100, startCallback);
  ros::Subscriber goal_sub = n.subscribe("goal", 100, goalCallback);

  // publish path to /path
  // to publish Path, create a vector of PoseStamped
  ros::Publisher path_pub = n.advertise<const nav_msgs::Path>("path", 1000);
  ros::spinOnce();

  // wait for map data
  while(mapData.data.size()<1){
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    ROS_INFO("No map data");
  }

  // wait for start and goal points
  while(!goal_received){
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    ROS_INFO("No goal provided");
  }

  // run RRT algo

  // // create temporary PoseStamped object for origin and goal 
  // geometry_msgs::PoseStamped start;
  // start.pose.position.x = 1.0;
  // start.pose.position.y = 1.0;
  // start.pose.position.z = 0.0;
  // start.header.frame_id = "map";
  // start.pose.orientation = tf::createQuaternionMsgFromYaw(0);

  // ROS_INFO("Created PoseStamped");
  // geometry_msgs::PoseStamped goal;
  // goal.pose.position.x = 490.0;
  // goal.pose.position.y = 490.0;
  // goal.pose.position.z = 0.0;
  // goal.header.frame_id = "map";
  // goal.pose.orientation = tf::createQuaternionMsgFromYaw(0);

  // initialise RRT class with mapData and points
  RRT rrt(mapData, start, goal);
  int goal_index = rrt.FindPath();
  ROS_INFO("goal index is: %d", goal_index);
  std::vector<geometry_msgs::PoseStamped> poses = rrt.BuildPath(goal_index, start, goal);
  ROS_INFO("number of points: %d", poses.size());
  
  nav_msgs::Path path;
  path.poses = poses;
  path.header.frame_id = "map";

  while(ros::ok()){
    path_pub.publish(path);
    r.sleep();
  }
  return 0;
}

