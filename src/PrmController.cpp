#include "PrmController.h"

/**
 * @brief Construct a new Prm Controller:: Prm Controller object
 * 
 * @param nh - ros node handle
 */
PrmController::PrmController(ros::NodeHandle nh)
    : ok_(true), debug_(false)
{
    init(nh);
    threads_.push_back(new std::thread(&PrmController::publishMap, this));
    threads_.push_back(new std::thread(&PrmController::updatePRM, this));
}

/**
 * @brief Destroy the Prm Controller:: Prm Controller object
 * 
 */
PrmController::~PrmController(void)
{
    // Signal threads to shut down
    ok_ = false;
    if (debug_)
    ROS_INFO_STREAM("Received shut down request");
    // Join threads before leaving
    for (auto t : threads_)
    {
        t->join();
    }
}

/**
 * @brief Callback function for the subscription to robot_0/odom
 * Captures the current robot pose.
 * 
 * @param msg - Contains robot pose
 */
void PrmController::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    prm_key_.lock();
    prm_.setRobotPose(msg->pose.pose);
    prm_key_.unlock();
    Robot_Pose_Recieved_ = true;
}

/**
 * @brief Callback function for the subscription to local_map/local_map
 * Captures the occupancy grid map
 * 
 * @param msg - Contains occupancy grid information
 */
void PrmController::mapCallback(const nav_msgs::OccupancyGridPtr& msg)
{   
    OGmap_key_.lock();
    prm_.setOGmap(msg->data,
        msg->info.height,
        msg->info.width,
        msg->info.resolution);
    OGmap_key_.unlock();    
}
/**
 * @brief Callback function for the subscription to move_base_simple/goal
 * Captures goal requests
 * 
 * @param msg - Contains requested goal pose
 */
void PrmController::goalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    // Set the goal pose
    prm_key_.lock();
    bool success = prm_.setGoalPose(msg->pose);
    prm_key_.unlock();
    // In case goal pose requested is invalid
    if (success)
    {
        Goal_pose_received_ = true;
        Path_found_ = false;
    }
}

/**
 * @brief Service that responds to robot_0/request_goal
 * This service receives a goal request in global co-ordinates,
 * tests the Prm's ability to reach that goal, and responds via
 * res.ack with a true or false answer.
 * Note: The return value indicates the service's ability to handle
 * the request, it does NOT indicate whether the goal can be reached.
 *   
 * @param req - requested goal pose
 * @param res - reference to a response variable that gets written
 *              with the result. 
 * @return true - successfully handled the request
 * @return false - unsuccessfully handled the request
 */
bool PrmController::requestGoal(a4_setup::RequestGoal::Request& req, a4_setup::RequestGoal::Response& res)
{
    prm_key_.lock();
    geometry_msgs::Pose goal;
    goal.position.x = req.pose.x;
    goal.position.y = req.pose.y;

    // Failure to set the goal pose indicates goal is in occupied area
    if (!prm_.setGoalPose(goal))
    {
        // Reply with failure and exit immediately 
        res.ack = false;
        return true;
    }
    // We must path find based on the valid set of nodes 
    // in case the ogmap has changed since nodes were inserted!
    prm_.ValidateRoadMap();
    res.ack = prm_.findPath();

    prm_key_.unlock();
    return true;
}

/**
 * @brief Thread that publishes a marker array to robot_0/prm.
 * Prm Nodes are published as Blue Cylinders.
 * Robot Pose & Goal Pose are published as red cyclinders.
 * Path to goal is published as a line-strip marker in green.
 * 
 */
void PrmController::publishMap(void)
{
    ros::Rate rate_limiter(5.0);
    static visualization_msgs::MarkerArray marker_array;

    // Create a unique locker so that we can use .wait()
    std::unique_lock<std::mutex> prm_locker(prm_key_);
    prm_locker.unlock();

    while(ok_)
    {
        marker_array.markers.clear();
        // Wait here for a new node to be published
        prm_cond_.wait(prm_locker);

        // Add all of the nodes to the marker array
        std::vector<Node> current_nodes = prm_.getNodes();
        for (auto n : current_nodes)
        {
            marker_array.markers.push_back(defaultCylinder(n, Basic_Blue));
        }
        // Add in the robot postion
        // Node robot_node(32000, {prm_.getRobotPose().position.x, prm_.getRobotPose().position.y});
        // marker_array.markers.push_back(defaultCylinder(robot_node, Basic_Red));

        // Add in the goal position
        geometry_msgs::Pose goal_pose;
        if (prm_.requestGoalPose(goal_pose))
        {
            Node goal_node(32001, {goal_pose.position.x, goal_pose.position.y});
            marker_array.markers.push_back(defaultCylinder(goal_node, Basic_Red));
        }
        // Add in edges!
        if (Show_Edges_)
        {
            if (debug_)
                ROS_INFO_STREAM("Showing edges");
            std::vector<edge_pair> edge_pairs = prm_.getEdges();
            for (auto p : edge_pairs)
            {
                marker_array.markers.push_back(defaultEdge(p,Basic_Yellow));
            }
        }

        // Add in the path
        if (Path_found_)
        {
            std::vector<geometry_msgs::Pose> poses_in_route = prm_.getPosesToGoal();
            marker_array.markers.push_back(defaultStrip(poses_in_route,Basic_Green));
        }

        marker_pub_.publish(marker_array);
        if (debug_)
            ROS_INFO_STREAM("published prm markers");
        // Give up access to the prm
        // prm_cond_.notify_all();
        rate_limiter.sleep();

    }
    if (debug_)
        ROS_INFO_STREAM("publishMap thread shutting down");
}

/**
 * @brief This thread will waituntil a goal request is received
 * and then continuously spawn nodes within the current 
 * occupancy grid until a valid path is found to the goal.
 * 
 */
void PrmController::updatePRM(void)
{
    // Rate limiter is to slow down node creation.
    // Ideally I wouldn't have done this, but I kept ending up with
    // too many nodes and causing rviz to crash
    ros::Rate rate_limiter(25.0); 

    while(ros::ok && ok_)
    {
        // Ensure the map is available
        while (!Robot_Pose_Recieved_) 
        {
            if (debug_)
                ROS_INFO_STREAM("updatePRM Thread Sleeping");
            rate_limiter.sleep();
        }
        OGmap_key_.lock();
        prm_key_.lock();
        if (!Path_found_ && Goal_pose_received_)
        // if (!Path_found_)
        {
            bool success = false;
            unsigned int attempts =0;
            // Put in a timeout for node creation if the map is saturated
            while (!success && attempts < node_create_timeout)
            {
                // Generate a new node
                success = prm_.addValidNode();
                attempts++;
            }

            if (debug_)
            {
                if (success)
                    ROS_INFO_STREAM("Success creating a new node!");
                else
                    ROS_INFO_STREAM("Failed to create a new node"); 
            }

            // We must path find based on the valid set of nodes 
            // in case the ogmap has changed since nodes were inserted!
            prm_.ValidateRoadMap();
            Path_found_ = prm_.findPath();
            if (Path_found_)
            {
                if (debug_)
                    ROS_INFO_STREAM("Path Found!!!!");
                publishPath();
            }
        }
        prm_key_.unlock();
        OGmap_key_.unlock();

        // Signal new data for the consumer thread
        prm_cond_.notify_all();
        rate_limiter.sleep();

    }
    if (debug_)
        ROS_INFO_STREAM("updateMap thread shutting down");
    prm_key_.unlock();
    OGmap_key_.unlock();
}

/**
 * @brief Publishes the path to the move_base_simple/goal topic.
 * This function gets called whenever the path to a new goal is
 * discovered. Unlike the prm publisher, the path is only published
 * once, which is why publishPath is a simple function and not a thread.
 * 
 */
void PrmController::publishPath(void)
{
    if (Path_found_)
    {
        // Create a pose array and fill it with the poses to goal
        geometry_msgs::PoseArray pa;
        std::vector<geometry_msgs::Pose> poses_in_route = prm_.getPosesToGoal();
        // Add in the goal pose!
        geometry_msgs::Pose goal_pose;
        if (prm_.requestGoalPose(goal_pose))
        {
            poses_in_route.push_back(goal_pose);
        }
        pa.poses = poses_in_route;
        // Add in tiemstamp
        pa.header.stamp = ros::Time::now();
        path_pub_.publish(pa);
    }
}

void PrmController::init(ros::NodeHandle nh)
{
    nh_ = nh;
    // Set up subscriptions
    odom_sub_ = nh_.subscribe("robot_0/odom",1, &PrmController::odomCallback, this);
    goal_sub_ = nh_.subscribe("move_base_simple/goal", 1, &PrmController::goalCallback, this);
    map_sub_ = nh_.subscribe("local_map/local_map", 1, &PrmController::mapCallback, this);
    // Set up publishers
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("robot_0/prm",3);
    path_pub_ = nh_.advertise< geometry_msgs::PoseArray>("/robot_0/path",3);
    // Set up service
    goal_service = nh_.advertiseService("robot_0/request_goal", &PrmController::requestGoal, this);

    ros::NodeHandle priv("~");
    RosConfig default_config = Default_RosPrm_Config;
    // max neighbours
    priv.param<int>("max_neighbours",       config_.max_neighbours,     default_config.max_neighbours); 
    // max neighbour distance
    priv.param<double>("max_neighbour_dist",config_.max_neighbour_dist, default_config.max_neighbour_dist); 
    // show edges between all nodes
    priv.param<bool>("show_edges",          config_.show_edges,         default_config.show_edges); 
    // min node seperation
    priv.param<double>("node_seperation",   config_.node_seperation,    default_config.node_seperation); 
    // max reattempts at creating node
    priv.param<int>("max_node_retry",       config_.max_node_retry,     default_config.max_node_retry); 
    // number of scans per pixel
    priv.param<int>("scan_per_pixel",       config_.scan_per_pixel,     default_config.scan_per_pixel); 
    // marker duration 
    priv.param<double>("marker_lifetime",   config_.marker_lifetime,    default_config.marker_lifetime);
    // Now set all of these values
    priv.param<bool>("debug_msgs",        config_.debug_msgs,    default_config.debug_msgs);
    commandHandle();
}

void PrmController::commandHandle(void)
{
    // max neighbours
    prm_.setMaxNeighbours(config_.max_neighbours);
    // max neighbour distance
    prm_.setNeighbourRadius(config_.max_neighbour_dist);
    // min node seperation
    prm_.setMinNodeSpacing(config_.node_seperation);
    // number of scans per pixel
    prm_.setScansPerPixel(config_.scan_per_pixel);
    // show edges between all nodes
    Show_Edges_ = config_.show_edges;
    // max reattempts at creating node
    node_create_timeout = config_.max_node_retry;
    // How long markers last on screen
    marker_duration_ = config_.marker_lifetime;
    // turn on messages
    debug_ = config_.debug_msgs;
}