
#ifndef NODE_H
#define NODE_H

#include <utility>

// Nodes are initalised without parents
#define NO_PARENT __UINT32_MAX__
// useful definition for node identification 
typedef unsigned int id_t;

/**
 * @brief global x-y co-ordinate container
 * I wanted this class to be completely agnostic to
 * ros data-types so ros_msgs::Point was avoided here 
 * although it could perfectly replace the globalOrd struct.
 * 
 */
struct globalOrd
{
    double x;
    double y;
};

/**
 * @brief The node class is to hold important information required for graphs.
 *  A node essentially consists of an id & global xy position, along with 
 *  a few other helpful properties for graph searches.
 * 
 */
class Node
{
    public:
        // Constructors 
        Node(){};
        Node(unsigned int id, globalOrd pos);
        // setters
        void setParent(unsigned int parent_id);
        void setRadius(double r);
        // getters
        unsigned int getParent(void);
        unsigned int getID(void);
        globalOrd getPosition(void);
        double getRadius();
        // Public methods
        void incrementNeighbours(void);
        void decrementNeighbours(void);
        unsigned int countNeighbours(void);

    private:
        // private variables
        /**
         * @brief unique identifier
         * 
         */
        id_t id_;
        /**
         * @brief id of this nodes 'parent'.
         * 
         */
        id_t parent_;  
        /**
         * @brief the position of this node in global co-ordinates
         * 
         */
        globalOrd position_;
        /**
         * @brief the radius of this node
         * 
         */
        double radius_;
        /**
         * @brief the number of neighbours this node has
         * 
         */
        unsigned int n_neighbours_;
};

/**
 * @brief Useful for creating 'edges', which are links between two nodes
 * 
 */
typedef std::pair<Node, Node> edge_pair;

#endif
