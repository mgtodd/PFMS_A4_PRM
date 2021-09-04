#include <gtest/gtest.h>
#include <iostream>
#include <vector>
#include <algorithm>

#include "../src/graph.h"
#include "../src/node.h"

TEST(SimpleDijkstraSearch, skipNodes24)
{
    id_t goal = 5;
    id_t start = 1;
    Graph g;

    g.insertNode(0,2);
    g.insertNode(1,2);
    g.insertNode(1,1);
    g.insertNode(2,1);
    g.insertNode(3,0);

    std::vector<id_t> bestRoute {1,3,5};
    bool success = g.Dijkstra(start,goal);
    EXPECT_EQ(true,success);
    std::vector<Node> route = g.getNodesToGoal();
    std::vector<id_t> route_id;
    for (auto n : route)
    {
        route_id.push_back(n.getID());
    }
    EXPECT_EQ(bestRoute,route_id);
}

TEST(Neighbours, out_of_range)
{
    id_t goal = 5;
    id_t start = 1;
    Graph g;

    double unit = DEFAULT_NHBR_RADIUS -0.1;

    g.insertNode(0,0);
    g.insertNode(unit,0);
    g.insertNode(unit,unit);
    
    std::vector<id_t> expected = {2};
    EXPECT_EQ(expected, g.getNeighbours(1));
    std::vector<id_t> expected2 = {1,3};
    EXPECT_EQ(expected2, g.getNeighbours(2));
    std::vector<id_t> expected3 = {2};
    EXPECT_EQ(expected3, g.getNeighbours(3));
}

TEST(Neighbours, tooManyNeighbours)
{
    id_t goal = 5;
    id_t start = 1;
    Graph g;

    // ensure that all nodes will be close enough to the centre to form an edge
    double unit = DEFAULT_NHBR_RADIUS/5.0;

    g.insertNode(0,0);
    g.insertNode(unit,0);
    g.insertNode(unit,unit);
    g.insertNode(0,unit);
    g.insertNode(-unit,unit);
    g.insertNode(-unit,0);
    g.insertNode(-unit,-unit);
    
    std::vector<id_t> expected = {2,3,4,5,6};
    EXPECT_EQ(expected, g.getNeighbours(1));
    // std::vector<id_t> expected2 = {1,3};
    // EXPECT_EQ(expected2, g.getNeighbours(2));
    // std::vector<id_t> expected3 = {2};
    // EXPECT_EQ(expected3, g.getNeighbours(3));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
