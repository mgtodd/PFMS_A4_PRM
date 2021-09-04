#include "graph.h"
#include <iostream>

#include <set>
#include <queue>
#include <map>
#include <cstring>
#include <cmath>

/**
 * @brief Construct a new Graph:: Graph object
 * 
 */
Graph::Graph(void)
    : node_count_(0),
      route_possible_(false),
      max_neighbours_(DEFAULT_MAX_NHBRS),
      neighbour_radius_(DEFAULT_NHBR_RADIUS),
      min_node_spacing(DEFAULT_NODE_SPACING)
      {}

/**
 * @brief 
 * -Attempts to insert a node into the graph, failing if the
 *  requested position is too close to other nodes
 * -This function will automatically create edges with 
 *  neighbouring nodes.
 * -This is the reccommended way to add nodes to the graph!
 * 
 * @param x global x-position of node
 * @param y global y-position of node
 * @return true - successful node insertion
 * @return false - unssuccessful node insertion
 */
bool Graph::insertNode(double x, double y)
{
    // If we are too close to another existing node, do not add this node to the graph
    if (!enforceMinNodeDist(globalOrd {x,y}))
        return false;
    // Generate a unique ID for the node
    Node n(unique_id(),globalOrd {x,y});
    // Create a vertex out of this node
    Vertex v(n);
    graph_.insert({n.getID(),v});
    // Locate the neighbours of this new node and create appropriate edges 
    discoverNeighbours(n.getID());
    return true;
}

/**
 * @brief Overloaded copy of insertNode to allow insertion with a node object
 * 
 */
bool Graph::insertNode(Node n)
{
    double x = n.getPosition().x;
    double y = n.getPosition().y;
    return insertNode(x,y);
}

/**
 * @brief Enforces minimum distance requirement
 * 
 * @param p - global co-orinates of the node being tested
 * @return true - position meets distance constraint
 * @return false - position fails distance constraint
 */
bool Graph::enforceMinNodeDist(globalOrd p)
{
    globalOrd nhbr_pos;
    double dist;
    // Compare node to all other nodes
    for (int nhbr = node_count_; nhbr > 0; nhbr--)
    {
        nhbr_pos = graph_.at(nhbr).node.getPosition();
        // Pythagoras
        dist = sqrt(pow((p.x - nhbr_pos.x),2) + pow((p.y - nhbr_pos.y),2));

        // If we find an existing node that is too close, immediately return failure
        if (dist <= min_node_spacing)
        {
            return false;
        }
    }
    // If we reach this point, minimum distance constraints have been met
    return true;
}
/**
 * @brief Finds the closest node to a given point
 * 
 * @param p p - global co-orinates of the point being tested
 * @return id_t - id of the closest node to the point
 */
id_t Graph::closestNodeToPoint(globalOrd p)
{
    double dist, ref_dist = neighbour_radius_;
    // If there are no neighbours closer than neighbour_radius_, return zero
    id_t closest = 0;
    globalOrd nhbr_pos;
    // Compare node to all other nodes
    for (int nhbr = node_count_; nhbr > 0; nhbr--)
    {
        nhbr_pos = graph_.at(nhbr).node.getPosition();
        // Pythagoras
        dist = sqrt(pow((p.x - nhbr_pos.x),2) + pow((p.y - nhbr_pos.y),2));

        // If we find a closer node, update the reference distance and closest id
        if (dist < ref_dist)
        {
            ref_dist = dist;
            closest = nhbr;
        }
    }
    return closest;
}
/**
 * @brief Calculate the distance between two nodes
 * 
 * @param src - id of first node
 * @param nhbr - id of second node
 * @return double - distance between the two nodes
 */
double Graph::distBetweenNodes(id_t src, id_t nhbr)
{
    globalOrd p1 = graph_.at(src).node.getPosition();
    globalOrd p2 = graph_.at(nhbr).node.getPosition();
    // Shoutout to Pythagoras
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));    
}

/**
 * @brief Locates nodes within neighbour_radius_ of given node 
 *  and creates appropriate edges.
 * 
 * @param src - id of subject node
 */
void Graph::discoverNeighbours(id_t src)
{
    // Any new node is going to have an id higher than every other, so we can simply compare to each id lower than src
    id_t nhbr = src -1;
    Node n2, n1 = graph_.at(src).node;
    
    while (nhbr > 0)
    {
        double dist = distBetweenNodes(src, nhbr);

        // Ensure distance is within bound & neighbour limit not exceeded
        n2 = graph_.at(nhbr).node;  
        if (dist < neighbour_radius_ && n1.countNeighbours() < max_neighbours_)
        {
            if (n2.countNeighbours() < max_neighbours_)
            {
                // Create an edge between these 2 points
                addEdge(src,nhbr,dist);
            }
            else // Ensure we are only keeping the 5 closest nodes
            {
                edges_t nhbr_edges = graph_.at(nhbr).edges;
                weight furthest_dist = 0;
                id_t furthest_nhbr = 0;
                // Locate the neighbour that is furthest from this node
                for (auto e : nhbr_edges)
                {
                    if (e.second > furthest_dist)
                    {
                        furthest_dist = e.second;
                        furthest_nhbr = e.first;
                    }
                }
                // Check if the candidate node is closer than
                // one of nhbr's neighbours
                if (dist < furthest_dist)
                {
                    // Replace this Neighbour
                    // First we delete the edge at nhbr and furthest_nhbr
                    graph_.at(nhbr).edges.erase         (graph_.at(nhbr).edges.find(furthest_nhbr));
                    graph_.at(furthest_nhbr).edges.erase(graph_.at(furthest_nhbr).edges.find(nhbr));
                    graph_.at(nhbr).node.decrementNeighbours();
                    graph_.at(furthest_nhbr).node.decrementNeighbours();

                    // Delete from the unique edges list as well
                    for (auto it = unique_edges_.begin(); it != unique_edges_.end(); it++)
                    {
                        if (it->first.getID() == nhbr && it->second.getID() == furthest_nhbr ||
                            it->second.getID() == nhbr && it->first.getID() == furthest_nhbr)
                        {
                            unique_edges_.erase(it);
                            break;
                        }
                    }

                    // Now we add an edge between src and nhbr
                    addEdge(src,nhbr,dist);
                }

            }
            

        } 
        // move the neighbour id we are checking
        nhbr--;
    }
}

/**
 * @brief generates a unique id for a node
 * 
 * @return id_t - unique id
 */
id_t Graph::unique_id(void)
{
    node_count_++;
    return node_count_;
}

/**
 * @brief Ensures the graph contains a node with this id
 * 
 * @param id - id to be tested
 * @return true - if node exists
 * @return false - if node does not exist
 */
bool Graph::hasId(id_t id)
{
    // Id's are unique so this will return a 1 or a 0
    return graph_.count(id);
}

/**
 * @brief Checks if an edge exists between two nodes
 * 
 * @param src - id of the first node
 * @param dest - id of the second node
 * @return true - if edge exists
 * @return false - if edge does not exist
 */
bool Graph::hasEdge(id_t src, id_t dest)
{
    // Ensure vertices exist
    if (!this->hasId(src) || !this->hasId(dest))
        return false;

    // check if the src has an edge containing the dest
    int count = graph_.at(src).edges.count(dest);
    return (bool)count;
}

/**
 * @brief Retrieves the neighbours of a given node
 * 
 * @param src - id of subject node
 * @return std::vector<id_t> - a vector containing the id of all neighborus
 */
std::vector<id_t> Graph::getNeighbours(id_t src)
{
    std::vector<id_t> neighbours;
    // Ensure this node exists
    if (!this->hasId(src))
        return neighbours;

    // // Extract the neighbour id's from the map of edges relating to the src
    edges_t edges = graph_.at(src).edges;
    for (auto  e : edges)
    {
        neighbours.push_back(e.first);
    }

    return neighbours;
}

/**
 * @brief gets the weight of the edge between 2 nodes
 * 
 * @param src - first node being queried
 * @param dest - second node being queried
 * @return weight - weight of edge between the two nodes
 */
weight Graph::getWeight(id_t src, id_t dest)
{
    // Ensure these nodes exist
    if (!this->hasId(src) || !this->hasId(dest))
        return MAXFLOAT;
    edges_t edges = graph_.at(src).edges;
    weight w = edges.at(dest);
    return w;
}

/**
 * @brief Grabs a copy of all the nodes in the graph
 * 
 * @return std::vector<Node> - all nodes in the graph
 */
std::vector<Node> Graph::getNodes(void)
{
    std::vector<Node> nodes;
    id_t idx = node_count_;

    // Iterate across the whole graph and take each node
    while (idx > 0)
    {
        nodes.push_back(graph_.at(idx).node);
        idx--;
    }
    return nodes;
}

/**
 * @brief Returns a reference to the list of edges
 *  The intended use is so that the edges can be validated
 *  with respect to constraints beyond the scope of the
 *  graph class.
 *  This function is called by the prm class so that edges
 *  passing through occupied space in the ogmap can be deleted.
 * 
 * @return std::vector<edge_pair>& 
 */
std::vector<edge_pair>& Graph::getEdges(void)
{
    return unique_edges_;
}

/**
 * @brief returns a reference to the graph_t object.
 *  The intended use is so that the edges can be validated
 *  with respect to constraints beyond the scope of the
 *  graph class.
 *  This function is called by the prm class so that edges
 *  passing through occupied space in the ogmap can be deleted.
 * 
 * @return graph_t& -reference to the graph_t object.
 */
graph_t& Graph::getGraph(void)
{
    return graph_;
}

/**
 * @brief calculates the 'radius' of this node from the start node 
 *  'radius' refers to the total weight of the path taken to get
 *  back to the start.
 * @param child - node whose radius is being queried
 * @param first_via - the first node in the route back to the
 *                    start node. This allows testing alternate 
 *                    routes.
 * @return weight - the total distance of the path from child
 *                  back to the start node via first_via.
 */
weight Graph::calculateRadius(id_t child, id_t first_via)
{
    if (graph_.at(first_via).node.getParent() == child)
    {
        return 10000000;
    }
    // std::cout<< "enter calcRad\n
    weight radius = 0;
    // Get the node directly above
    id_t parent = first_via;
    Node childnode = graph_.at(child).node;
    Node parentnode = graph_.at(parent).node;
    // Find the radius from the start node
    while (parent != child) // The start node has the special property that its parent == itself
    {  
        // Find the weight of this edge
        radius += getWeight(parent,child);
        // move the child (traverse the tree back towards root)
        child = parent;
        // Get the node directly above
        parent = graph_.at(child).node.getParent();

        childnode = graph_.at(child).node;
        parentnode = graph_.at(parent).node;
    }
    // std::cout<< "exit calcRad\n";
    return radius;
}

/**
 * @brief Attempts to construct the route from the start node 
 * to goal. Result is stored in route_to_goal_, which
 * contains a vector of nodes in the route and the 
 * 'radius' of the route.
 * - Note: The dijkstra search will not necessarily find a path 
 *  to every node (if no edges exist leading to it), so the 
 *  requested goal is not necessarily reachable.        
 * 
 * @param goal - id of route destination
 * @return true - if it is possible to reach goal
 * @return false - if not possible to reach goal
 */
bool Graph::buildRoute(id_t goal)
{
    // std::cout<< "enter buildroute\n";
    // Graph::route_t route;
    std::vector<id_t> nodes_in_route;
    nodes_in_route.push_back(goal);

    id_t child = goal;

    auto parent = graph_.at(child).node.getParent();
    // Find the radius from the start node
    // The start node has the special property that its parent == itself
    while (parent != child && parent != NO_PARENT) 
    {  
        // Add to the vector of nodes on the route
        nodes_in_route.push_back(parent);
        // move the child (traverse the tree back towards root)
        child = parent;
        // Get the node directly above
        parent = graph_.at(child).node.getParent();
    }

    if (parent == NO_PARENT)
    {
        // There is no possible route to the goal
        route_possible_ = false;
    }
    else
    {   
        route_possible_ = true;
        // Due to the design of the Dijkstra search, the first node in the vector is actually the furthest from the start. We will flip the vector
        std::vector<id_t> corrected_nodes;
        for (int i = nodes_in_route.size() -1; i >= 0; i--)
        {
            corrected_nodes.push_back(nodes_in_route.at(i));
        }
        route_to_goal_.first = corrected_nodes;
        
        route_to_goal_.second = graph_.at(goal).node.getRadius();
    }
    // std::cout<< "exit buildroute\n";
    return route_possible_;
}

/**
 * @brief Retrieves all nodes in the route to the goal
 * 
 * @return std::vector<Node> 
 */
std::vector<Node> Graph::getNodesToGoal(void)
{
    std::vector<id_t> ids_in_route = route_to_goal_.first;
    std::vector<Node> nodes_in_route;
    for (auto id : ids_in_route )
    {
        nodes_in_route.push_back(graph_.at(id).node);
    }
    return nodes_in_route;
}

/**
 * @brief Creates an edge between two nodes
 * 
 * @param src - id of first node in edge
 * @param dest - id of second node in edge
 * @param weight - weight of this edge
 */
void Graph::addEdge(id_t src, id_t dest, double weight)
{
    // Ensure these nodes exist
    if (!this->hasId(src) || !this->hasId(dest) || this->hasEdge(src,dest))
        return;

    graph_.at(src).edges.insert({dest,weight});
    graph_.at(dest).edges.insert({src,weight});

    addUniqueEdge(src,dest);

    graph_.at(src).node.incrementNeighbours();
    graph_.at(dest).node.incrementNeighbours();
}

/**
 * @brief Adds to a unique list of edges
 * 
 * @param src - id of first node in edge
 * @param dest - id of second node in edge
 */
void Graph::addUniqueEdge(id_t src, id_t dest)
{
    edge_pair ep(graph_.at(src).node,graph_.at(dest).node);
    unique_edges_.push_back(ep);
}   

/**
 * @brief Performs a dijkstra search on the graph to find the quickest
 *  path from start -> goal.
 *  -Starting from start, we traverse the graph breadth-first.
 *  -When a new node is discovered, the node who discovered it 
 *   is set as its 'parent' and its radius is calculated.
 *  -If the node is rediscovered by another node, we check whether it 
 *   is faster to reach this node from the discovering node. If so, the 
 *   discovering node becomes its parent.
 *  -Using this method, the best path to every node that can be reached
 *  is discovered, all that is needed is to trace parent by parent 
 *  back toward the start.
 * 
 * @param start 
 * @param goal 
 * @return true 
 * @return false 
 */
bool Graph::Dijkstra(id_t start, id_t goal)
{
    // Perform a Dijkstra search to find the best path start->finish   

    // BestPathToNode_.push_back(Graph::path {start,"-",0});
    graph_.at(start).node.setParent(start);
    graph_.at(start).node.setRadius(0);

    // Stores id of nodes that are waiting to be searched
    std::queue<id_t> wait_list;

    // Stores nodes that have been discovered
    std::set<id_t> discovered;

    wait_list.push(start);
    discovered.insert(start);

    while (wait_list.size() > 0)
    {
        // get the neighbours from this node
        auto neighbours = getNeighbours(wait_list.front());
        for (auto node_id : neighbours)
        {
            // If we reach the end, then we have not discovered this node
            if (discovered.find(node_id) == discovered.end())
            {
                // std::cout << "new node discovered\n";
                // We have a new unique node
                wait_list.push(node_id);
                discovered.insert(node_id);
                // Add this node and specify its via as the current node (who found it)
                // BestPathToNode_.push_back(path {node,wait_list.front(),__INT_MAX__});
                graph_.at(node_id).node.setParent(wait_list.front());
                // Find the radius from start via the 'parent' who discovered this new node
                graph_.at(node_id).node.setRadius(calculateRadius(node_id,wait_list.front()));
            }
            else // We have found a new path to this node
            {
                // Graph::path current_path = getPath(node);
                unsigned int new_radius = calculateRadius(node_id,wait_list.front());

                if ( new_radius < graph_.at(node_id).node.getRadius() )
                {
                    // std::cout << "new best path discovered\n";

                    //Ensure that we do not make 2 nodes point at each other
                    if (graph_.at(wait_list.front()).node.getParent() == node_id)
                    {
                        // IGNORE
                        auto x = 1;
                    }
                    else
                    {
                        graph_.at(node_id).node.setParent(wait_list.front());
                        graph_.at(node_id).node.setRadius(new_radius);                        
                    }
                    
                }
            }
            
        }
        wait_list.pop();
    }

    return buildRoute(goal);

}
/**
 * @brief Configure the maximum allowed neighbours for each node
 * 
 * @param max 
 */
void Graph::setMaxNeighbours(unsigned int max)
{
    max_neighbours_ = max;
}

void Graph::setNeighbourRadius(double r)
{
    neighbour_radius_ = r;
}
void Graph::setMinNodeSpacing(double k)
{
    min_node_spacing = k;
}

