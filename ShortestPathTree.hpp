#ifndef SHORTEST_PATH_TREE_HPP
#define SHORTEST_PATH_TREE_HPP

#include "SpanningTree.hpp"
#include <vector>

/**
 * @brief ShortestPathTree class
 * 
 * @author Jonathan Ung
 */
class ShortestPathTree : SpanningTree {
    private:
        std::vector<int> costs;
    public:
        ShortestPathTree();
        ShortestPathTree(int, std::vector<int>, std::vector<int>, std::vector<int>);
        int getCost(int);
};

#endif