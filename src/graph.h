
#ifndef GRAPH_H
#define GRAPH_H

#include "node.h"

#include <vector>
#include <map>


#define DEFAULT_MAX_NHBRS 5
#define DEFAULT_NHBR_RADIUS 3.0
#define DEFAULT_NODE_SPACING 0.25

/**
 * @brief a weight assigns a value to a connection
 * between two nodes
 * 
 */
typedef double weight;
/**
 * @brief nodes are accessed and specified via
 * their id number
 * 
 */
typedef unsigned int id_t;
/**
 * @brief the edges_t datatype maps the id of
 * the neighbourign node to a weight
 * 
 */
typedef std::map<id_t, weight> edges_t;

/**
 * @brief Vertex structure contains a node and edges
 * 
 */
struct Vertex
{
    Node node;
    edges_t edges;

    inline Vertex(Node node) : node(node) {}
};
/**
 * @brief The graph_t datatype maps a node id to a Vertex
 * 
 */
typedef std::map<id_t, Vertex> graph_t;

/**
 * @brief Pairs a vector of nodes, comprising a 'route' 
 * to the total weight of the route
 * 
 */
typedef std::pair<std::vector<id_t>, weight> route_t;

/**
 * @brief A weighted graph class. 
 * The underlying graph_t object is instantiated to map
 * each node to a set of edges.
 * Class contains a Dijkstra search method to find the quickest route
 * between two points.
 * 
 */
class Graph
{
    // See graph.cpp for documentation
    public: 
        // Constructor
        Graph();
        // Setters
        void setMaxNeighbours(unsigned int max);
        void setNeighbourRadius(double r);
        void setMinNodeSpacing(double k);
        bool insertNode(double x, double y);
        bool insertNode(Node n);
        void addEdge(id_t src, id_t dest, double weight);
        // Getters
        std::vector<Node> getNodes(void);
        std::vector<edge_pair>& getEdges(void);
        graph_t& getGraph(void);
        std::vector<Node> getNodesToGoal(void);
        std::vector<id_t> getNeighbours(id_t src);
        //Public Methods    
        bool Dijkstra(id_t start, id_t goal);
        id_t closestNodeToPoint(globalOrd point);

    private: 
        /**
         * @brief map each node to a set of edges.
         * 
         */
        graph_t graph_;
        /**
         * @brief The route to the goal, which is a mapping
         * of a vector of nodes and a combined weight
         * 
         */
        route_t route_to_goal_;
        /**
         * @brief indicates whether the most recent dijkstra
         * search found a route to the requested goal.
         * 
         */
        bool route_possible_;
        /**
         * @brief a count of the number of nodes in the graph.
         * Used to generate unique id's
         * 
         */
        id_t node_count_;
        /**
         * @brief As edges get created within the graph_t object
         * they are also copied into this vector. This allows
         *  all the edges contained in the graph to be quickly
         * grabbed: the vector would otherwise need to be
         * constructed at the time of request.  
         * 
         */
        std::vector<edge_pair> unique_edges_;
        /**
         * @brief the maximum allowed neighbours for a given node
         * 
         */
        unsigned int max_neighbours_;

        /**
         * @brief Furthest distance twos node can be apart and still 
         * form an edge.
         * 
         */
        double neighbour_radius_;
        
        /**
         * @brief Minimum distance between nodes.
         * 
         */
        double min_node_spacing;
        
        // Private methods
        id_t unique_id(void);
        double distBetweenNodes(id_t src, id_t dest);
        void addUniqueEdge(id_t, id_t);
        bool enforceMinNodeDist(globalOrd p);
        void discoverNeighbours(id_t src);
        bool hasId(id_t);
        bool hasEdge(id_t src, id_t dest);
        bool buildRoute(id_t goal);
        weight getWeight(id_t src, id_t dest);
        weight calculateRadius(id_t child, id_t first_via);
};

#endif