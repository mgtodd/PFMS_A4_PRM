#include "transform.h"

/**
 * @brief Construct a new Transform:: Transform object
 * 
 */
Transform::Transform(void)
{}

/**
 * @brief Update important map information
 * 
 * @param map - ogmap
 * @param height - map height in pixels
 * @param width - map width in pixels
 * @param res - map resolution is metres
 */
void Transform::UpdateMap(std::vector<int8_t> map,
                unsigned int height,
                unsigned int width,
                float res)
{
    OGmap_ = map;
    map_height_ = height;
    map_width_ = width;
    resolution_=  res;
}

/**
 * @brief Updates the robot pose
 * 
 * @param rbp - new robot pose
 */
void Transform::UpdateRobotPose(geometry_msgs::Pose rbp)
{
    robot_pose_ = rbp;
}

/**
 * @brief Test whether a global point is in unoccupied space
 * 
 * @param global - global point being tested
 * @return true - this point is in unoccupied space
 * @return false - this point is not in unoccupied space
 */
bool Transform::isGlobalFree(geometry_msgs::Pose global)
{
    localPixel local;
    if (GlobalToOg(global,local))
    {
        // Returns 1 if local is free
        return(isLocalFree(local));
    }
    else
    {
        // Failing to convert globaltoOg means the
        // point is not within the occupancy grid!
        return 0;
    }

}

/**
 * @brief Overloaded copy of function to facilitate
 * using a node as argument
 */
bool Transform::isGlobalFree(Node n)
{
    geometry_msgs::Pose p;
    p.position.x = n.getPosition().x;
    p.position.y = n.getPosition().y;
    p.position.z = 0;
    return isGlobalFree(p);
}

/**
 * @brief Converts a point in the occupancy grid, which is 
 * in robot-centric co-ordinates to a point in global xy
 * co-ordinates. 
 * 
 * @param local - xy point w.r.t robot to be transformed
 * @return geometry_msgs::Pose - global space co-ordinate equivalent
 */
geometry_msgs::Pose Transform::OgToGlobal(localPixel local)
{
    // The local pixel is a point on the occupancy grid map confined to (+-100,+-100) We wish to transform this into a pose with global span.

    // We use the robot pose as a reference
    geometry_msgs::Pose transformed;
    transformed.position.x = 
        robot_pose_.position.x + (float)local.x * resolution_;
    transformed.position.y = 
        robot_pose_.position.y + (float)local.y * resolution_;
    transformed.position.z = 0.0;

    return transformed;
}

/**
 * @brief Converts a global xy co-ordinate to a robot-centric
 * co-ordinate. If the requested global co-ordinate is beyond
 * the boundaries of the occupancy grid the conversion is not 
 * possible and the function returns false.
 * 
 * @param global - global co-ordinate to be transformed
 * @param local - reference to xy point w.r.t robot. Gets written
 *                if the conversion is possible
 * @return true - successful conversion
 * @return false - unsuccessful conversion - point outside ogmap
 */
bool Transform::GlobalToOg(geometry_msgs::Pose global, localPixel& local)
{
    // The global span can contain points that are not necessarily within the robot's local occupancy grid. In this case, we just return 0,0
    double upper_x = robot_pose_.position.x + resolution_ * (map_width_/2 - 1);
    double lower_x = robot_pose_.position.x - resolution_ * (map_width_/2 - 1);
    double upper_y = robot_pose_.position.y + resolution_ * (map_height_/2 - 1);
    double lower_y = robot_pose_.position.y - resolution_ * (map_height_/2 - 1);

    // Apply constraints
    if (global.position.x  > upper_x ||
        global.position.y > upper_y ||
        global.position.x  < lower_x ||
        global.position.y < lower_y)
    {
        //Invalid point requested
        return 0;
    }
    // Succesful conversion
    local.x = (int16_t)round((global.position.x - robot_pose_.position.x)/resolution_);
    local.y = (int16_t)round((global.position.y - robot_pose_.position.y)/resolution_);
    return 1;
}   

/**
 * @brief tests whether a point in the occupancy grid is
 * in unoccupied space
 * 
 * @param local - robot-centric xy co-ordinate being considered
 * @return true - pixel is in unoccupied space
 * @return false - pixel is in either occupied of undefined space
 */
bool Transform::isLocalFree(localPixel local)
{
    // Finds the occupancy of a pixel within the map
    // -1 indicates unknown, 0-100 represents %occupancy
    uint16_t map_idx = 
        ( (local.y + map_height_ / 2) * map_width_ +
        (local.x + map_width_ / 2) )  ;

    if (map_idx > map_width_*map_height_)
    {
        // ERROR
        return 0;
    }
    // Return 1 if free
    return (OGmap_.at(map_idx) == 0);
}

/**
 * @brief Validates a connection between two global co-ordinates.
 * A line is constructed between the two points and continuously tested
 * to make sure no points are in occupied / undefined space 
 * 
 * @param x1 - x co-ordinate of first point
 * @param y1 - y co-ordinate of first point
 * @param x2 - x co-ordinate of second point
 * @param y2 - y co-ordinate of second point
 * @return true - all points between two points are free
 * @return false - obstacle detected between two points
 */
bool Transform::checkConnection(double x1, double y1, double x2, double y2)
{
    //Create coordinates that will scan down line
    //Normalise delta x for distance of 1, then multiply by scan length
    double dist_12 = sqrt(pow((x1-x2),2) + pow((y1-y2),2));
    double x_inc = (x2 - x1) / dist_12 * (resolution_ / scans_per_pixel);
    double y_inc = (y2 - y1) / dist_12 * (resolution_ / scans_per_pixel);

    //How many scan lengths can fit in total distance, not including start and end scans
    int num_increments = dist_12 / (resolution_ / scans_per_pixel); 

    //Check points on the line
    for(int i = 1; i <= num_increments; i++){
        geometry_msgs::Pose targ;
        targ.position.x = x1 + (i * x_inc);
        targ.position.y = y1 + (i * y_inc);
        localPixel local;
        if( GlobalToOg(targ,local))
        {
            if (!isLocalFree(local))
            {
                //Fail if any point has a collision
                return false; 
            }        
        }
    }

    return true;
}

/**
 * @brief Overloaded copy function to facillate testing
 * of edge-pairs
 */
bool Transform::checkConnection(edge_pair edges)
{
    // Extract the two node postions and pass in
    Node n1 = edges.first;
    Node n2 = edges.second;
    double x1 = n1.getPosition().x;
    double y1 = n1.getPosition().y;
    double x2 = n2.getPosition().x;
    double y2 = n2.getPosition().y;

    return checkConnection(x1,y1,x2,y2);
}
/**
 * @brief get the current map height
 * 
 * @return unsigned int 
 */
unsigned int Transform::getHeight(void)
{
    return map_height_;
}
/**
 * @brief Get the current map width
 * 
 * @return unsigned int 
 */
unsigned int Transform::getWidth(void)
{
    return map_width_;
}

/**
 * @brief Set scans per pixel
 * 
 * @param k 
 */
void Transform::setScansPerPixel(unsigned int k)
{
    scans_per_pixel = k;
}
