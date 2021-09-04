#include <gtest/gtest.h>
#include <iostream>
#include <vector>
#include <algorithm>

#include "../src/transform.h"

static const int OCCUPIED = 100;
static const int FREE = 0;
static const int height = 200;
static const int width = 200;
static const float resolution = 0.1;

TEST(Connectivity, vertical)
{
    Transform transformer;

    std::vector<int8_t> ogmap;
    // Create an ogmap with a solid line down the middle
    for (int row =0; row < width; row++)
    {
        for (int col = 0; col < height; col++)
        {   
            if (col == 100)
                ogmap.push_back(OCCUPIED);
            else
                ogmap.push_back(FREE);
        }
    }
    // Set the robot to 0,0
    geometry_msgs::Pose robot_pose;
    robot_pose.position.x = 0;
    robot_pose.position.y = 0;
    robot_pose.position.z = 0;

    // Set the map
    transformer.UpdateMap(ogmap,height,width,resolution);
    transformer.UpdateRobotPose(robot_pose);

    // Verify points just either side of the vertical line as free
    double x1 = 1;   
    double y1 = 0;
    double x2 = -1;
    double y2 = 0;
    geometry_msgs::Pose p1;
    p1.position.x = x1;
    p1.position.y = y1;
    bool testp1 = transformer.isGlobalFree(p1);
    EXPECT_EQ(true,testp1);
    geometry_msgs::Pose p2;
    p2.position.x = x2;
    p2.position.y = y2;
    bool testp2 = transformer.isGlobalFree(p2);
    EXPECT_EQ(true,testp2);
    // Verify that the origin is occupied
    geometry_msgs::Pose origin;
    origin.position.x = 0;
    origin.position.y = 0;
    bool testOrigin = transformer.isGlobalFree(origin);
    EXPECT_EQ(false,testOrigin);

    // Try to go through the vertical line! this should fail
    bool can_connect = transformer.checkConnection(x1,y1,x2,y2);
    EXPECT_EQ(false,can_connect);
    // Try to go through backwards
    can_connect = transformer.checkConnection(x2,y1,x1,y2);
    EXPECT_EQ(false,can_connect);

    // If we move the robot now to 3,0 the veritcal line will move to
    // x = 3
    // Therefore, a connection from (-1,0) to (1,0) should be possible
    robot_pose.position.x = 3;
    transformer.UpdateMap(ogmap,height,width,resolution);
    transformer.UpdateRobotPose(robot_pose);
    can_connect = transformer.checkConnection(x1,y1,x2,y2);
    EXPECT_EQ(true,can_connect);
}

TEST(GlobalToOg,simple)
{
    Transform transformer;

    std::vector<int8_t> ogmap;
    // Create an empty ogmap 
    for (int row =0; row < width; row++)
    {
        for (int col = 0; col < height; col++)
        {   
            ogmap.push_back(FREE);
        }
    }
    // Set the robot to 0,0
    geometry_msgs::Pose robot_pose;
    robot_pose.position.x = 0;
    robot_pose.position.y = 0;
    robot_pose.position.z = 0;

    // Set the map
    transformer.UpdateMap(ogmap,height,width,resolution);
    transformer.UpdateRobotPose(robot_pose);

    geometry_msgs::Pose p1;
    p1.position.x = 6.3;
    p1.position.y = -3.3;
    localPixel expected = {63,-33};
    localPixel result;
    transformer.GlobalToOg(p1,result);


    // x limit
    p1.position.x = 10.0;
    p1.position.y = 0;
    bool possible =transformer.GlobalToOg(p1,result);
    EXPECT_EQ(possible,false);

    // y limit
    p1.position.x = 0;
    p1.position.y = 10.0;
     possible = transformer.GlobalToOg(p1,result);
    EXPECT_EQ(possible,false);

    // Request a point within bounds with the robot not at zero
    robot_pose.position.x = 3;
    robot_pose.position.y = 3;
    transformer.UpdateMap(ogmap,height,width,resolution);
    transformer.UpdateRobotPose(robot_pose);

    p1.position.x = 12;
    p1.position.y = 0;
    expected = {90,-30};
    possible = transformer.GlobalToOg(p1,result);
    EXPECT_EQ(possible,true);
    EXPECT_EQ(expected.x,result.x);
    EXPECT_EQ(expected.y,result.y);

    p1.position.x = -21.04;
    p1.position.y = -6.43;

    robot_pose.position.x = -19.24;
    robot_pose.position.y = -6.6;
    transformer.UpdateRobotPose(robot_pose);
    // expected = {90,-30};
    possible = transformer.GlobalToOg(p1,result);
    EXPECT_EQ(possible,true);
}


TEST(Connectivity, Slanted)
{
    Transform transformer;

    std::vector<int8_t> ogmap;
    // Create an ogmap with a slanted line going SW corner to NE corner
    for (int row =0; row < width; row++)
    {
        for (int col = 0; col < height; col++)
        {   
            if (row == col)
                ogmap.push_back(OCCUPIED);
            else
                ogmap.push_back(FREE);
        }
    }
    // Set the robot to 0,0
    geometry_msgs::Pose robot_pose;
    robot_pose.position.x = 0;
    robot_pose.position.y = 0;
    robot_pose.position.z = 0;

    // Set the map
    transformer.UpdateMap(ogmap,height,width,resolution);
    transformer.UpdateRobotPose(robot_pose);

    // Try to go through the line veritcally down: this should fail
    bool can_connect = transformer.checkConnection(0.0,1.0,0.0,-1.0);
    EXPECT_EQ(false,can_connect);
    // Try to go through the line veritcally upwards
    can_connect = transformer.checkConnection(0.0,-1.0,0.0,1.0);
    EXPECT_EQ(false,can_connect);

    // Try to go through the line horizontally: this should fail
    can_connect = transformer.checkConnection(-1.0,0.0,1.0,0.0);
    EXPECT_EQ(false,can_connect);
    // Try to go through the line horizontally backwards
    can_connect = transformer.checkConnection(1.0,0.0,-1.0,0.0);
    EXPECT_EQ(false,can_connect);

    // Try to go through the line at 45 degrees
    can_connect = transformer.checkConnection(-1.0,-1.0,1.0,1.0);
    EXPECT_EQ(false,can_connect);
    // Try to go through the line at -45 degrees
    can_connect = transformer.checkConnection(-1.0,1.0,1.0,-1.0);
    EXPECT_EQ(false,can_connect);

    // Try to go through the line with small angle
    can_connect = transformer.checkConnection(-1.2,-0.9,1.2,0.9);
    EXPECT_EQ(false,can_connect);
    // Try to go through the line at  - small angle
    can_connect = transformer.checkConnection(-1.2,0.9,1.2,-0.9);
    EXPECT_EQ(false,can_connect);

}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}