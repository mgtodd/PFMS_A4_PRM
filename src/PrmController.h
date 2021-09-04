#ifndef PRMCONTROLLER_H
#define PRMCONTROLLER_H

#include "RosInterface.h"
#include "prm.h"

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

#include <mutex>
#include <condition_variable>

#define DEBUG

/**
 * @brief Class designed to interface a Prm with ros.
 * 
 */
class PrmController : public RosInterface
{
    public: 
        // Constructors/ Destructors
        PrmController(ros::NodeHandle nh);
        ~PrmController();
        // Subscriptions / service callbacks
        void odomCallback(const nav_msgs::OdometryConstPtr& msg);
        void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg);
        void mapCallback(const nav_msgs::OccupancyGridPtr& msg);
        bool requestGoal(a4_setup::RequestGoal::Request& req, a4_setup::RequestGoal::Response& res);
        void init(ros::NodeHandle nh);
        void commandHandle(void);
    private:
        // Variables   
        const RosConfig Default_RosPrm_Config =
        {
            5,      // max neighbours
            3.0,    // max neighbour distance
            false,  // show edges between all nodes
            0.25,   // min node seperation
            1000,   // max reattempts at creating node
            4,      // number of scans per pixel
            1.0,    // marker lifetime
            false   // debug messages
        };
        RosConfig config_;

        unsigned int marker_count_;
        /**
         * @brief Probabilistic road map object. 
         * 
         */
        Prm prm_;
        /**
         * @brief Vector containing the threads. These threads are joined
         * in the class destructor.
         * 
         */
        std::vector<std::thread*> threads_;
        /**
         * @brief Mutex for access to the occupancy grid map
         * 
         */
        std::mutex OGmap_key_;
        /**
         * @brief Mutex for access to the prm
         * 
         */
        std::mutex prm_key_;
        /**
         * @brief Condition variable to allow the producer thread
         * (updatePRM) to signal new data to the consumer thread
         * (publishMap). 
         * 
         */
        std::condition_variable prm_cond_;
        /**
         * @brief Flag indicating whether robot pose received
         * 
         */
        bool Robot_Pose_Recieved_;
        /**
         * @brief Flag indicating whether goal pose received
         * 
         */
        bool Goal_pose_received_;
        /**
         * @brief Flag indicating whether path to goal has been found
         * 
         */
        bool Path_found_;
        /**
         * @brief Show the edges between nodes
         * 
         */
        bool Show_Edges_;
        /**
         * @brief Number of retries to create a vald node
         * 
         */        
        unsigned int node_create_timeout;
        /**
         * @brief Signifies that the program is ok to continue running
         * 
         */
        bool ok_;
        /**
         * @brief Turn on/off messages
         * 
         */
        bool debug_;
        // Threads 
        void publishMap(void);
        void updatePRM(void);

        // Private methods
        void publishPath(void);



};

#endif