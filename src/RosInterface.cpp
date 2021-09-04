#include "RosInterface.h"

#define DEFAULT_MARKER_DURATION 1
/**
 * @brief Construct a new Ros Interface:: Ros Interface object
 * 
 */
RosInterface::RosInterface(void) 
    : next_id_(UINT32_MAX), marker_duration_(DEFAULT_MARKER_DURATION)
{}

/**
 * @brief generate a unique id for a marker
 * 
 * @return id_t 
 */
id_t RosInterface::unique_marker_id(void)
{
    next_id_--;
    return next_id_;
}

/**
 * @brief Constructs a linestrip marker to connect 2 points with
 * default parameters. 
 * 
 * @param p - pair of nodes that will form the edge
 * @param col - marker colour
 * @return visualization_msgs::Marker
 */
visualization_msgs::Marker RosInterface::defaultEdge(edge_pair p, RGBtriple col)
{
    // Basic marker properties
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/world";      
    marker.header.stamp = ros::Time::now(); 
    marker.lifetime = ros::Duration(marker_duration_);
    marker.id = unique_marker_id();
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    // Add the two points
    geometry_msgs::Point p1;
    p1.x = p.first.getPosition().x;
    p1.y = p.first.getPosition().y;
    p1.z = 0.3;
    marker.points.push_back(p1);

    geometry_msgs::Point p2;
    p2.x = p.second.getPosition().x;
    p2.y = p.second.getPosition().y;
    p2.z = 0.3;
    marker.points.push_back(p2);

    marker.scale.x = 0.05;
    // Colour
    marker.color.a = 1;
    marker.color.r = col.r;
    marker.color.g = col.g;
    marker.color.b = col.b;

    return marker;
}

/**
 * @brief Generate a line strip with default values
 * 
 * @param poses - vector of poses that form the strip
 * @param col - marker colour
 * @return visualization_msgs::Marker 
 */
visualization_msgs::Marker RosInterface::defaultStrip(std::vector<geometry_msgs::Pose> poses, RGBtriple col)
{
    // Basic marker properties
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/world";      
    marker.header.stamp = ros::Time::now(); 
    marker.lifetime = ros::Duration(marker_duration_);
    marker.id = unique_marker_id();
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    // Add all of the node positions
    geometry_msgs::Point point;
    for (auto p : poses)
    {
        point.x = p.position.x;
        point.y = p.position.y;
        point.z = 0.3;
        marker.points.push_back(point);
    }

    marker.scale.x = 0.05;
    //Colour 
    marker.color.a = 1.0f;
    marker.color.r = col.r;
    marker.color.g = col.g;
    marker.color.b = col.b;

    return marker;
}

/**
 * @brief Generate a marker cylinder with default values
 * 
 * @param n 
 * @param col 
 * @return visualization_msgs::Marker 
 */
visualization_msgs::Marker RosInterface::defaultCylinder(Node n, RGBtriple col)
{
    // Create a marker
    visualization_msgs::Marker marker;
    // Generate a unique marker id
    marker.id = (int32_t)n.getID(); 
    marker.header.frame_id = "/world";      
    marker.header.stamp = ros::Time::now(); 
    marker.lifetime = ros::Duration(marker_duration_);
    marker.ns = "nodes";           
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = n.getPosition().x;
    marker.pose.position.y = n.getPosition().y;
    marker.pose.position.z = 0.2;

    //Orientation, we are not going to orientate it
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Marker colour
    marker.color.a = 1;
    marker.color.r = col.r;
    marker.color.g = col.g;
    marker.color.b = col.b;

    return marker;
}

