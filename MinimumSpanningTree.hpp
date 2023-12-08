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
        MinimumSpanningTree(int r, std::vector<int> &p, std::vector<int> &sO, double tW) : SpanningTree(r, p, sO) { totalWeight = tW; }
        ~MinimumSpanningTree() {}
        double getTotalWeight() { return totalWeight; }
};

#endif