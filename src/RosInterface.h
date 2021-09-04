#ifndef ROSINTERFACE_H
#define ROSINTERFACE_H

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"

// #include <cv_bridge/cv_bridge.h>
#include "a4_setup/RequestGoal.h"
#include "nav_msgs/OccupancyGrid.h"

#include <thread>
#include "node.h"

/**
 * @brief struct for rgb color creation
 * 
 */
struct RGBtriple
{
    double r;
    double g;
    double b;

    inline RGBtriple(double r, double g, double b) 
        : r(r), g(g), b(b) {}
};

/**
 * @brief A few basic colours
 * 
 */
static const RGBtriple Basic_Red    (1,0,0);
static const RGBtriple Basic_Green  (0,1,0);
static const RGBtriple Basic_Blue   (0,0,1);
static const RGBtriple Basic_Yellow (1,1,0);

/**
 * @brief Class designed to be a generic interface to ros.
 * 
 */
class RosInterface
{
    public:
        RosInterface();
        virtual void odomCallback(const nav_msgs::OdometryConstPtr& msg)= 0;
        virtual void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg)= 0;
        virtual void mapCallback(const nav_msgs::OccupancyGridPtr& msg)= 0;
        virtual bool requestGoal(a4_setup::RequestGoal::Request& req, a4_setup::RequestGoal::Response& res)= 0;
        virtual void init(ros::NodeHandle nh) = 0;
    protected:
        /**
         * @brief Ros Configuration Parameters that facilitate command line arguments
         * 
         */
        struct RosConfig {
        int max_neighbours;         // max neighbours
        double max_neighbour_dist;  // max neighbour distance
        bool show_edges;            // show edges between all nodes
        double node_seperation;     // min node seperation
        int max_node_retry;         // max reattempts at creating node
        int scan_per_pixel;         // number of scans per pixel
        double marker_lifetime;     // marker display timeout
        bool debug_msgs;            // turns on/off debugging messages
        };
        RosConfig config_;
        /**
         * @brief ros node handle for the PrmController
         * 
         */
        ros::NodeHandle nh_;
        /**
         * @brief subscriber to robot_0/odom
         * 
         */
        ros::Subscriber odom_sub_;
        /**
         * @brief subscriber to move_base_simple/goal
         * 
         */
        ros::Subscriber goal_sub_;
        /**
         * @brief subscriber to local_map/local_map
         * 
         */
        ros::Subscriber map_sub_;
        /**
         * @brief publisher to robot_0/prm
         * 
         */
        ros::Publisher marker_pub_;
        /**
         * @brief publisher to /robot_0/path
         * 
         */
        ros::Publisher path_pub_;
        /**
         * @brief serviceserver for /robot_0/request_goal
         * 
         */
        ros::ServiceServer goal_service;
        /**
         * @brief A count of the number of nodes created. Used to 
         * grant unique id's to each node.
         * 
         */
        double marker_duration_;
        visualization_msgs::Marker defaultCylinder(Node n, RGBtriple col);
        visualization_msgs::Marker defaultEdge(edge_pair p, RGBtriple col);
        visualization_msgs::Marker defaultStrip(std::vector<geometry_msgs::Pose> nodes, RGBtriple col);
    private:
        id_t unique_marker_id(void);
        unsigned int next_id_;

};

#endif