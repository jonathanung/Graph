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
        std::vector<double> costs;
    public:
        ShortestPathTree() {}
        ShortestPathTree(int r, std::vector<int> &p, std::vector<int> &sO, std::vector<double> &c) : SpanningTree(r, p, sO) { costs = c; }
        ~ShortestPathTree() {}
        int getCost(int i) { return costs[i]; }
};



#endif