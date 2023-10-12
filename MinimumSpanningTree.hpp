#ifndef MINIMUM_SPANNING_TREE_HPP
#define MINIMUM_SPANNING_TREE_HPP

#include "SpanningTree.hpp"
#include <vector>

/**
 * @brief MinimumSpanningTree class
 * 
 * @author Jonathan UNg
 */
class MinimumSpanningTree : SpanningTree {
    private:
        double totalWeight;
    public:
        MinimumSpanningTree();
        MinimumSpanningTree(int, std::vector<int>, std::vector<int>, double);
        ~MinimumSpanningTree() {}
        double getTotalWeight();
};

#endif