#include "node.h"
#include "nav_msgs/Odometry.h"

/**
 * @brief Local xy co-ordinates w.r.t robot.
 * This co-ordinate scheme is bounded as:
 * -map_width_ < x < map_width_ - 1
 * -map_height_ < y < map_height_ - 1
 * 
 */
struct localPixel
{
    int16_t x;
    int16_t y;
};
/**
 * @brief This class contains useful functions for conversions between
 * robot-centric and global co-ordinate schemes.
 * - The class object is to be loaded with information about 
 * the occupancy grid and robot position.
 * - In addition, this class can validate connections between two
 * points and find the occupancy of points within the ogmap.
 * 
 */
class Transform
{
    public:
        Transform();
        void UpdateRobotPose(geometry_msgs::Pose rbp);
        void UpdateMap(std::vector<int8_t> map,
                unsigned int height,
                unsigned int width,
                float res);
        void setScansPerPixel(unsigned int k);
        unsigned int getHeight();
        unsigned int getWidth();
        
        geometry_msgs::Pose OgToGlobal(localPixel local);
        bool GlobalToOg(geometry_msgs::Pose global, localPixel& local);
        bool isLocalFree(localPixel local);
        bool isGlobalFree(geometry_msgs::Pose global);
        bool isGlobalFree(Node n);
        bool checkConnection(double x1, double y1, double x2, double y2);
        bool checkConnection(edge_pair edges);

    private:
        /**
         * @brief The robot pose
         * 
         */
        geometry_msgs::Pose robot_pose_;
        /**
         * @brief The Occupancy grid map
         * 
         */
        std::vector<int8_t> OGmap_;
        /**
         * @brief The occupancy grid map height in pixels
         * 
         */
        unsigned int map_height_;
        /**
         * @brief The occupancy grid map width in pixels
         * 
         */
        unsigned int map_width_;
        /**
         * @brief The occupancy grid map resolution in metres.
         * Defines the width of 1 pixel in metres.
         * 
         */
        float resolution_;
        /**
         * @brief Number of scans per pixel, used for checking
         * connectivity.
         * 
         */
        unsigned int scans_per_pixel;
};