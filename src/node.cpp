#include "node.h"

/**
 * @brief Construct a new Node:: Node object
 * 
 * @param id - id of the new node
 * @param pos - node position in global xy
 */
Node::Node(id_t id, globalOrd pos)
    :   id_(id), 
        parent_(NO_PARENT),
        position_(pos), 
        radius_(0), 
        n_neighbours_(0)
{}

/**
 * @brief sets Parent of node
 * 
 * @param parent_id 
 */
void Node::setParent(id_t parent_id)
{
    parent_ = parent_id;
}

/**
 * @brief grabs id of node's parent
 * 
 * @return id_t 
 */
id_t Node::getParent(void )
{
    return parent_;
}

/**
 * @brief grabs the node id
 * 
 * @return id_t 
 */
id_t Node::getID(void )
{
    return id_;
}
/**
 * @brief returns the global xy coordinate of node
 * 
 * @return globalOrd 
 */
globalOrd Node::getPosition(void)
{
    return position_;
}
/**
 * @brief sets node radius
 * 
 * @param r 
 */
void Node::setRadius(double r)
{
    radius_ = r;
}
/**
 * @brief grabs radius
 * 
 * @return double 
 */
double Node::getRadius(void)
{
    return radius_;
}
/**
 * @brief increments the neighbour count
 * 
 */
void Node::incrementNeighbours(void)
{
    n_neighbours_++;
}
/**
 * @brief decrement the neighbour count
 * 
 */
void Node::decrementNeighbours(void)
{
    n_neighbours_--;
}
/**
 * @brief grabs the neighbour count
 * 
 * @return unsigned int 
 */
unsigned int Node::countNeighbours(void)
{
    return n_neighbours_;
}
