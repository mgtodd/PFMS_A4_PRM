#include "prm.h"

/**
 * @brief Construct a new Prm:: Prm object
 * 
 */
Prm::Prm(void)
{
    seed_ = std::chrono::system_clock::now().time_since_epoch().count();
}

/**
 * @brief Set robot pose
 * 
 * @param p - robot pose
 */
void Prm::setRobotPose(geometry_msgs::Pose p)
{
    tf_.UpdateRobotPose(p);
    robot_pose_ = p;
}

/**
 * @brief retrieve the robot pose
 * 
 * @return geometry_msgs::Pose 
 */
geometry_msgs::Pose Prm::getRobotPose(void)
{
    return robot_pose_; 
}   

/**
 * @brief sends map information to the transformer class object
 * 
 * @param map - occupancy grid map
 * @param height - map height in pixels
 * @param width - map width in pixels
 * @param res - the width of each pixel in metres
 */
void Prm::setOGmap(  std::vector<int8_t> map,
                unsigned int height,
                unsigned int width,
                float res)
{
    tf_.UpdateMap(map, height, width, res);
}

/**
 * @brief Attempts to set goal pose
 * 
 * @param p - requested goal pose
 * @return true - if goal pose reachable
 * @return false - if goal pose not reachable
 */
bool Prm::setGoalPose(geometry_msgs::Pose p)
{
    if (tf_.isGlobalFree(p))
    {
        goal_pose_set_ = true;
        goal_pose_ = p;
        return true;
    }
    else
        return false;
    
}
/**
 * @brief Attempt to retrieve the goal pose
 * 
 * @param p - a reference to a pose variable. Will be
 *          written if the goal pose has been set. 
 * @return true - if goal pose exists
 * @return false - if goal does not exist
 */
bool Prm::requestGoalPose(geometry_msgs::Pose& p)
{
    // Update the pose by reference if the pose has been set
    if (goal_pose_set_)
        p = goal_pose_;

    return goal_pose_set_;
}

/**
 * @brief Saturate a value to to ensure it is within 
 *        an upper and lower bound
 * 
 * @param val   - candidate value
 * @param lower - lower boundary
 * @param upper - upper boundary
 * @return int16_t - saturated value. Will be equal to 
 *              val if it val was within bounds.
 */
int16_t saturate(int16_t val,int16_t lower,int16_t upper)
{
    if (val > upper)
        val = upper;
    else if (val < lower)
    {
        val = lower;
    }
    return val;
}

/**
 * @brief grabs all the edges in the valid map
 * 
 * @return std::vector<edge_pair> 
 */
std::vector<edge_pair> Prm::getEdges(void)
{
    return valid_map_.getEdges();
    // return road_map_.getEdges();
}

/**
 * @brief Creates a new node within the unoccupied space
 * using a randomly generated robot-centric 
 * co-ordinates. 
 * -Due to the fact that there is a minimum-spacing between 
 * nodes enforced, this function will not necessarily 
 * succeed. 
 * 
 * @return true - if a new node successfully created
 * @return false - if no new node created
 */
bool Prm::addValidNode(void)
{
    // Use a standard normal distribution to spawn points
    static std::normal_distribution<double> distribution(0,1);
    static std::default_random_engine eng(seed_);

    bool isFree = false;
    localPixel pixel;

    unsigned int width = tf_.getWidth();
    unsigned int height = tf_.getHeight();
    while(!isFree)
    {   
        // Generate a random value for pixel location
        pixel = {(int16_t)(distribution(eng)*width / 2),
                 (int16_t)(distribution(eng)*height / 2)};

        // Saturate values at +-height/width
        pixel.x = saturate(pixel.x, -width/2, (width / 2) - 1);
        pixel.y = saturate(pixel.y, -height/2, (height / 2) -1);

        // Continue until we find an unoccupied point
        isFree = tf_.isLocalFree(pixel);
    }
    // Convert to global co-ords
    geometry_msgs::Pose xy = tf_.OgToGlobal(pixel);
    // Insert this node into the prm
    bool success = road_map_.insertNode(xy.position.x,xy.position.y);
    // Validate the edges with respect to the prm
    validateNewEdgesRoadMap();
    return success;
}

/**
 * @brief Ensures that edges created in the graph class
 *  are valid with respect to the occupancy grid map.
 *  By using a reference to the graph's edges we can
 *  delete edges that pass through occupied space!
 * 
 */
void Prm::validateNewEdgesRoadMap(void)
{
    std::vector<edge_pair>& edge_pairs = road_map_.getEdges();
    if (edge_pairs.size() == 0)
        return;
    
    auto stopping_point = edge_pairs.end()-6;
    for (auto it = edge_pairs.end()-1; it != stopping_point; it--)
    {
        // take care that we have not gone past the start
        if (it < edge_pairs.begin())
            break;
        // Ensure the edge only traverses unoccupied space
        if (!tf_.checkConnection(*it))
            edge_pairs.erase(it);
    }
}

/**
 * @brief Much like validateNewEdgesRoadMap except it uses
 *  validates edges for the valid map.
 *  I wish I could have consolidated these two functions,
 *  however I just could not get it to work modularly :(
 * 
 */
void Prm::validateNewEdgesValidMap(void)
{
    std::vector<edge_pair>& edge_pairs = valid_map_.getEdges();
    graph_t& graph = valid_map_.getGraph();
    if (edge_pairs.size() == 0)
        return;
    
    auto stopping_point = edge_pairs.end()-25;
    unsigned int count = 0;
    for (auto it = edge_pairs.end()-1; it != stopping_point; it--)
    {
        count++;
        // take care that we have not gone past the start
        if (it < edge_pairs.begin())
            break;
        // Ensure the edge only traverses unoccupied space
        if (!tf_.checkConnection(*it))
        {
            id_t id_1 =it->first.getID();
            id_t id_2 =it->second.getID();
            // remove edge pairs
            edge_pairs.erase(it);
            // also remove the edge from the graph
            graph.at(id_1).edges.erase(graph.at(id_1).edges.find(id_2));
            graph.at(id_2).edges.erase(graph.at(id_2).edges.find(id_1));
        }
    }

}

/**
 * @brief Creates a 'valid' prm with respect to the current occupancy grid.
 *  road_map_ contains all of the nodes ever created and all the possible edges
 *  between these nodes (count unrestricted).
 *  Instead of throwing these out when the road_map changes, they are simply
 *  ignored.
 *  The valid map contains only nodes and edges that are verfied to be in free space
 *  w.r.t the current ogmap.
 * 
 */
void Prm::ValidateRoadMap(void)
{
    std::vector<Node> allNodes = road_map_.getNodes();
    // Flush the current valid map
    Graph empty;
    valid_map_ = empty;

    // Validate the list of nodes against the current ogmap information
    for (auto n : allNodes)
    {
        // Returns 1 if point is free
        if (tf_.isGlobalFree(n))
        {
            valid_map_.insertNode(n);
            validateNewEdgesValidMap();
        }
    }
}

/**
 * @brief Finds the start and end points for the graph search.
 *  Since the robot pose and the goal pose are not added to 
 *  the search, we instead start at the robot's closest node
 *  and end at the closest node to the goal.
 * 
 * @param start - id of the start node, written via reference
 * @param end - id of the end node, written via reference
 * @return true - if there are start&end nodes close enough to robot&goal
 * @return false - if there are start&end nodes close enough to robot&goal
 */
bool Prm::findEndPoints(id_t& start, id_t& end)
{
    // Since the robot pose and the goal pose are not allowed on the map, we instead choose the start & end as the closest nodes available
    globalOrd robot_position = {robot_pose_.position.x, robot_pose_.position.y};
    start = valid_map_.closestNodeToPoint(robot_position);
    globalOrd goal_position = {goal_pose_.position.x, goal_pose_.position.y};
    end = valid_map_.closestNodeToPoint(goal_position);

    // if the id is 0 then there was no node within the min node distance
    if (start == end || start == 0 || end == 0)
        return false;
    else
        return true;
    
}
/**
 * @brief Attempts to find a path to the goal using the validmap
 * 
 * @return true - if path found
 * @return false - if path not found
 */
bool Prm::findPath(void)
{
    id_t start, end;
    // Find the end points of the road map
    bool possible = findEndPoints(start,end);
    if (!possible)
        return 0;

    // Perform search using Dijkstra
    return valid_map_.Dijkstra(start,end);
}

/**
 * @brief grab vector of poses from start to goal
 * 
 * @return std::vector<geometry_msgs::Pose> 
 */
std::vector<geometry_msgs::Pose> Prm::getPosesToGoal(void)
{
    // Get the nodes leading to the goal
    std::vector<Node> nodes_in_route = valid_map_.getNodesToGoal();
    // We want to transform these nodes into poses
    std::vector<geometry_msgs::Pose> poses_in_route;
    // Create poses out of the nodes 
    geometry_msgs::Pose p;
    for (auto n : nodes_in_route)
    {
        // Copy position, disregard orientation
        p.position.x = n.getPosition().x;
        p.position.y = n.getPosition().y;
        p.position.z = 0;
        poses_in_route.push_back(p);
    }
    // Add in the goal pose!
     geometry_msgs::Pose goal_pose;
    if (requestGoalPose(goal_pose))
        poses_in_route.push_back(goal_pose);
    return poses_in_route;

} 

/**
 * @brief get all of the nodes in the prm
 * 
 * @return std::vector<Node> 
 */
std::vector<Node> Prm::getNodes(void)
{
    return valid_map_.getNodes();
    // return road_map_.getNodes();
}

/**
 * @brief set Maximum number of neighbours a node can have
 * 
 */
void Prm::setMaxNeighbours(unsigned int m)
{
    // The road map is unbounded, the valid map is bounded
    road_map_.setMaxNeighbours(1000);
    valid_map_.setMaxNeighbours(m);
}

/**
 * @brief Set Maximum distance to form an edge beteween nodes
 * 
 */
void Prm::setNeighbourRadius(double r)
{
    road_map_.setNeighbourRadius(r);
    valid_map_.setNeighbourRadius(r);
}

void Prm::setMinNodeSpacing(double k)
{
    road_map_.setMinNodeSpacing(k);
    valid_map_.setMinNodeSpacing(k);
}


/**
 * @brief Set the number of scans used in continuity
 * checking
 * 
 */
void Prm::setScansPerPixel(unsigned int k)
{
    tf_.setScansPerPixel(k);
}
