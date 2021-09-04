#include <gtest/gtest.h>
#include <climits>
#include <cmath>

#include "ros/ros.h"
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "nav_msgs/OccupancyGrid.h"


#include "../src/prm.h"

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

nav_msgs::OccupancyGridPtr extractGrid(std::string file_name){

	std::string path = ros::package::getPath("a4_prm");
	path += "/test/bags/";
	std::string file = path + file_name;

	rosbag::Bag bag;
	bag.open(file);

	nav_msgs::OccupancyGridPtr grid;

	for(rosbag::MessageInstance const m: rosbag::View(bag))
	{
		grid = m.instantiate<nav_msgs::OccupancyGrid>();
		if (grid != nullptr){
		// Now we have the first grid in bag so we can proceed
		break;
		}
	}

	bag.close();

	return grid;
}

TEST(PathFind,double_goal_no_recalc)
{
    nav_msgs::OccupancyGridPtr grid = extractGrid("large_corridors_mutli_rooms.bag");

	ASSERT_NE(grid, nullptr);//Check that we have a grid from the bag

    Prm p;
    p.setMaxNeighbours(5);
    p.setNeighbourRadius(4.0);
    p.setMinNodeSpacing(0.25);
    p.setScansPerPixel(4);

    //Set the map
    p.setOGmap(grid->data, grid->info.height,grid->info.width,grid->info.resolution);
    // Position the robot was at when this capture was taken
    geometry_msgs::Pose rbp;
    rbp.position.x = -12;
    rbp.position.y = 0;
    p.setRobotPose(rbp);
    // A point in the next room - reachable but there are some obstacles in the way
    geometry_msgs::Pose goalPose;
    goalPose.position.x = -16.0;
    goalPose.position.y = -5.0;
    p.setGoalPose(goalPose);
    bool path_found = 0;
    // Set a timeout of 1000 iterations (which implies we have tried to add 1000 nodes)
    unsigned int timeout = 0;
    while (!path_found && timeout < 1000)
    {
        bool success = false;
        while (!success)
        {
            // Generate a new node
            success = p.addValidNode();
        }
        p.ValidateRoadMap();
        path_found = p.findPath();
        timeout++;
    }
    // Assert that we can find the path
    ASSERT_EQ(path_found,true);

    std::vector<geometry_msgs::Pose> first_path = p.getPosesToGoal();
    int sz1 = first_path.size();
    // Reinsert the same goal.
    // The Prm should be able to find the path again without adding any new nodes
    p.setGoalPose(goalPose);
    timeout = 0;
    while (!path_found && timeout < 1000)
    {
        p.ValidateRoadMap();
        path_found = p.findPath();
        if (path_found)
            break;
        timeout++;
        bool success = false;
        while (!success)
        {
            // Generate a new node
            success = p.addValidNode();
        }
    }
    // Assert that we can find the path
    ASSERT_EQ(path_found,true);

    std::vector<geometry_msgs::Pose> second_path = p.getPosesToGoal();
    int sz2 = second_path.size();

    ASSERT_EQ(sz1,sz2);
    if (sz1 == sz2)
    {
        // Check that the path nodes are all the exact same!
        for (int i =0; i<sz1;i++)
        {
            ASSERT_EQ(first_path.at(i).position.x,second_path.at(i).position.x);
            ASSERT_EQ(first_path.at(i).position.y,second_path.at(i).position.y);
        }

    }

}

TEST(PathFind,test3locations)
{
    nav_msgs::OccupancyGridPtr grid = extractGrid("large_open_spaces.bag");

	ASSERT_NE(grid, nullptr);

    Prm p;
    p.setMaxNeighbours(5);
    p.setNeighbourRadius(3.0);
    p.setMinNodeSpacing(0.25);
    p.setScansPerPixel(4);

    //Set the map
    p.setOGmap(grid->data, grid->info.height,grid->info.width,grid->info.resolution);
    // Position the robot was at when this capture was taken
    geometry_msgs::Pose rbp;
    rbp.position.x = 3.75;
    rbp.position.y = 1.47;
    p.setRobotPose(rbp);
    // A point in the corner of the map 
    geometry_msgs::Pose goalPose;
    goalPose.position.x = -3.0;
    goalPose.position.y = -7.0;
    p.setGoalPose(goalPose);
    bool path_found = 0;
    // Set a timeout of 1000 iterations (which implies we have tried to add 1000 nodes)
    unsigned int timeout = 0;
    while (!path_found && timeout < 1000)
    {
        bool success = false;
        while (!success)
        {
            // Generate a new node
            success = p.addValidNode();
        }
        p.ValidateRoadMap();
        path_found = p.findPath();
        timeout++;
    }
    // Assert that we can find the path
    ASSERT_EQ(path_found,true);

    // Change the goal to another corner of the map
    goalPose.position.x = -5.0;
    goalPose.position.y = 3.0;
    p.setGoalPose(goalPose);
    path_found = 0;
    // Set a timeout of 1000 iterations (which implies we have tried to add 1000 nodes)
    timeout = 0;
    while (!path_found && timeout < 1000)
    {
        bool success = false;
        while (!success)
        {
            // Generate a new node
            success = p.addValidNode();
        }
        p.ValidateRoadMap();
        path_found = p.findPath();
        timeout++;
    }
    // Assert that we can find the path
    ASSERT_EQ(path_found,true);

    goalPose.position.x = 10.0;
    goalPose.position.y = 3.0;
    p.setGoalPose(goalPose);
    path_found = 0;
    // Set a timeout of 1000 iterations (which implies we have tried to add 1000 nodes)
    timeout = 0;
    while (!path_found && timeout < 1000)
    {
        bool success = false;
        while (!success)
        {
            // Generate a new node
            success = p.addValidNode();
        }
        p.ValidateRoadMap();
        path_found = p.findPath();
        timeout++;
    }
    // Assert that we can find the path
    ASSERT_EQ(path_found,true);

}