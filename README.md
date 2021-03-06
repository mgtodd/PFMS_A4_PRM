# Launching the project
Launching the project requires the pfms_support package and the a4 root folder to exist in the workspace.
Before using the path follower we need to edit .bashr :
```
echo "source <full_path_to_git>/skeleton/a4_pf_install/setup.bash" >> /.bashrc 
```
This lets ROS know where the program resides.

To build the project, we can simply call 
```
catkin_make
```
from the catkin workspace folder located at  ~/catkin_ws

Now in order to run the project we need to first launch the simulator using
``` 
roslaunch a4_setup a4_setup.launch
 ```
then the a4_prm software using
```
 rosrun a4_prm a4_prm-run 
 ```
Rviz is a useful tool for visualisation, and can be launched anytime after the simulation
``` 
rviz -d $(rospack find a4_setup)/rviz/pfms/rviz 
```
Additionally, to launch the path following software we can use
```
 rosrun pfms_path_following pfms_path_following-velocity_control 
 ```


# Configuration 
The a4_prm project has a number of ros parameters available for configuration. These allow you
to quickly change the program behaviour without the need to recompile! The parameters are:
 - max_neighbours
 - max_neighbour_dist
 - show_edges
 - node_seperation
 - max_node_retry
 - scan_per_pixel
 - marker_lifetime
 - debug_msgs

For more information: See Documentation for RosInterface 

You can set these parameters while launching the program, for example we can set the max_neighbours
to 5 with the following syntax
```
 rosrun a4_prm a4_prm-run _max_neighbours:=5 
 ```
Note: There is a leading underscore before the varname.

We could also set parameters through the command line before running the program
``` 
rosparam set /a4_prm/max_neighbours 5 
```

# Code
The main functionality of the project occurs within the Prm class. This class contains member variables that
instantiate graphs, nodes and functional transforms for Global & Occupancy Grid data. I have designed the code
classes such that everything 'beneath' the prm level (graph & node) know nothing of ROS and ROS datatypes.
Transform & Prm use ROS datatypes, and the classes 'above' prm are designed to facilitate ROS communications: 
subscriptions, publishings and services.

# RosInterface
RosInterface is designed to be a generic interface to ros. This function provides virtual functions to allow for
configurable responses to callbacks and publishings. 
This class also offers several functions to simplify the creation of markers.

# PrmController
The PrmController is the interface for our prm to ROS. This class inherits from Rosinterface to use its generic
framework as a basis for its implementation with respect to the Prm.
Subscriptions:
 - Robot Position on /robot_0/odom
 - Occupancy Grid on /local_map/local_map and
 - Goal Requests  on /move_base_simple/goal
Publishings:
 - Path on /robot_0/path as a line-strip marker
 - Publishes Prm node locations (blue) & goal location (red) on /robot_0/prm as cylinders
Services:
 - Goal requests on /robot_0/request_goal

This class also primarily consists of 2 running threads:
 - PrmController::updatePRM (producing data)
 - PrmController::publishMap (consuming data)

For detailed description on the subscribers, publishers, service & threads see documentation for PrmController

# Prm
The Prm class considers information about the robot's environment and will attempt to find a path leading to a 
given goal pose.Path finding is facilitated by spawning randomly generated nodes within the map, creating edges 
where appropriate, until a path is discovered.

The Prm receives functionality from the graph class and the transform class via member variable instantiations.

The Prm contains 2 Graph::Graph objects: road_map_ and valid_map_. \n
The road_map_ graph contains all of the nodes that have ever been created. At the time of creation, each node is verified to be 
within free space within the occupancy grid. However, if the occupancy grid changes (robot moves), some or all of these 
nodes could potentially be in occupied/ undefined space. This makes them unusable for path searching, however they are 
kept in case they can be used again in the future. \n
valid_map_ is ensured to contain only nodes that can be verified as being in free space w.r.t the CURRENT Occupancy grid.
Therefore, valid_map is used for path searching.

# Transform
The Transform Class is used to transform between robot-centric and global co-ordinate systems. Essentially, transform is
used to help with important calculations for the prm.
This class also contains functionality for testing the occupancy of points in og or global space, and continuity tests
between two points, required when trying to form an edge between two nodes in a graph.

# Graph
The Graph class implements a weighted graph. The graph consists of nodes with a given x,y coordinate and edges which are
a link between two neighbour nodes with an associated weight. The weight of the edge represents the distance between the 
two nodes.\n 
The reccomended way to add nodes to the graph is via insertNode(). This will automatically discover nodes within radius
and create edges to them. \n
This class can find the shortest path between two nodes using a Dijkstra search. The Dijkstra search will actually uncover
the fastest route to every reachable node in the graph. Once finished, we can get the route from any goal back to the start.
Detailed information on the Dijkstra search can be found in graph documentation.

# Node
Node is a simple class used as the basic building block for a graph.
A node consists of a unique identifier and a position in global co-ordinates.
When the node is used in the context of the Graph, a few other properties are given meaning: \n
'Parent' refers to the first node in the path back towards root.
'Radius' refers to the total weight of the path to get back to root.

\section Unit Tests
Several unit tests have been written to test functionality of different elements of code.
 - test_graph
 - test_transform
 - test_prm_build

The tests are all compiled in the same makefile as the a4_prm project, you can compile with simply
``` 
catkin_make 
```

Some of these tests use rosbags to load the occupancy grid map information. These were recorded
by setting a scenario in ros stage and running 
``` 
rosbag record /tf /local_map/local_map /robot_0/odom 
```

# Simple unit testing
test_graph contains several tests for the functionality of the graph class. The graph class has been
designed so that it can be tested with minimal dependencies.
tests include attempting to create a neighbour after the neighbour limit is set, checking
edge-creation based on range & a very simple dijkstra search.

test_transform contains several tests to validate testing of points in global and
robot-centric co-ordinates as well as several tests for checking connectivity between 2 points.
In these tests, the ogmap is artifically constructed in code.

# Advanced unit testing
within test_prm_build I demonstrate D/HD features of unit testing. 
The test PathFind:double_goal_no_recalc demonstrates the prm's ability to reuse the existing ogmap 
when a new goal is set. By re-sending the same goal position, the prm is able to find the path again 
without needing to create new nodes

within test3locations, 3 different corners of the map are requested one after another as goal poses.
This test shows how the prm can can use pre-existing nodes and additionally create more until the goal is reached

In these tests, the ogmap information is extracted from a rosbag.
