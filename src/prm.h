#ifndef PRM_H 
#define PRM_H

// #define OCC_THRESH 50
#define LINE_SCAN_DEFINITION 4

#include "nav_msgs/Odometry.h"
#include "graph.h"
#include "node.h"
#include "transform.h"

#include <chrono>
#include <random>

/**
 * @brief The Prm class considers information about the robot's
 * environment and will attempt to find a path leading to a given
 * goal pose.
 * Path finding is facilitated by spawning randomly generated nodes
 * within the map until a path is discovered.
 * 
 */
class Prm
{
    public:
        Prm(void);
        // Setters
        void setMaxNeighbours(unsigned int m);
        void setNeighbourRadius(double r);
        void setScansPerPixel(unsigned int k);
        void setMinNodeSpacing(double k);
        void setRobotPose(geometry_msgs::Pose p);
        bool setGoalPose(geometry_msgs::Pose p);
        void setOGmap( std::vector<int8_t> map,
                    unsigned int height,
                    unsigned int width,
                    float res);       
        // Getters
        geometry_msgs::Pose getRobotPose(void);
        bool requestGoalPose(geometry_msgs::Pose& p);
        std::vector<Node> getNodes(void);
        std::vector<edge_pair> getEdges(void);
        // Public Methods
        bool addValidNode(void);
        bool findPath(void);
        std::vector<geometry_msgs::Pose> getPosesToGoal(void);
        void ValidateRoadMap();


    private:
        /**
         * @brief The robot pose in global co-ordinates
         * 
         */
        geometry_msgs::Pose robot_pose_;
        /**
         * @brief The goal pose in global co-ordinates
         * 
         */
        geometry_msgs::Pose goal_pose_;
        /**
         * @brief Flag to indicate whether a goal has been set.
         * 
         */
        bool goal_pose_set_;
        /**
         * @brief Graph that contains all the nodes that have been 
         * created since the program starts. May potentially 
         * contain nodes that are not necessarily in free space 
         * with respect to the CURRENT occupancy grid, however
         * at the time of insertion the node is valid. 
         * This is so that when the robot leaves an area of the map
         * and later comes back, it does not need to rediscover 
         * valid places to put nodes!
         * 
         */
        Graph road_map_;
        /**
         * @brief Graph that is valid with respect to the current 
         * occupancy grid. Every node & edge within this graph 
         * reside within and pass through free space.
         * 
         */
        Graph valid_map_;
        /**
         * @brief Transform class object, containing useful transforms
         * between global & robot-centric co-ordinates
         * 
         */
        Transform tf_;
        /**
         * @brief Random engine seed, intialised in constructor with
         * system time.
         * 
         */
        unsigned int seed_;

        //methods      
        bool findEndPoints(id_t& start, id_t& end);
        void validateNewEdgesRoadMap(void);
        void validateNewEdgesValidMap(void);
};

#endif