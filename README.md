# Rapidly-Exploring Random Tree Path Planner on ROS

## Dependencies 
1. ROS Noetic
2. Catkin
3. Rviz


## Usage 
1. To clone the package into your workspace and compile it

```
cd [workspace/src]
git clone https://github.com/sittingsotong/ROS-RRT
cd ..
catkin_make
source /devel/setup.bash
```

2. To launch path finding and map generation:
```
roslaunch rrt rrt.launch
```

3. To provide a start and end point, publish a ```geometry_msgs::PoseStamped``` message for each point to the topics ```/start``` and ```/goal``` respectively.

4. To visualise the path, check the rviz panel opened with the launch file. Add by topics ```/map``` and ```/path``` to see the data.



### YAML file
To select which map to run the test on, enter the ```maps``` directory and open the ```map.yaml``` file. 

```image``` indicates the path to the map that you want to use.

```resolution```: indicates the resolution per grid of the map. A resolution of 1 means each grid of the map represents 1m.

```origin```: indicates the point of the map to use as the origin.

```occupied_thresh```: indicates the minimum probability until the grid is considered occupied.

```free_thresh```: indicates the maximum probability before the grid is no longer free.

```negate```: is 1 if black spaces are free spaces.



### RRT Parameters 
Some parameters of the algoritm can be adjusted to better fit different types of maps. 

```max_iterations```: Maximum number of iterations before the algorithm decides that a path cannot be found. Default is 100000.

```goal_radius```: Distance away from the goal point before a path between the goal and node counts.

```step_size```: The size of a step the algorithm uses to test for an obstacle. A smaller size is more accurate while a larger size is faster.


## Issues 
1. Only works for resolution = 1. A different resolution causes the map and path sizes to be different when visualising it on rviz.
2. ```RRT::IsValid``` and ```IndexOfPoint``` functions in ```rrt.cpp``` does not seem to be working properly. Published path still cuts through obstacles. 
3. On some runs the algorithm cannot find a path. Rerunning it would usually work. 