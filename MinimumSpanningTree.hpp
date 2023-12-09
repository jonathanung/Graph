#ifndef MINIMUM_SPANNING_TREE_HPP
#define MINIMUM_SPANNING_TREE_HPP

#include "SpanningTree.hpp"
#include <vector>

/**
 * @brief MinimumSpanningTree class
 * 
 * @author Jonathan Ung
 */
class MinimumSpanningTree : SpanningTree {
    private:
        double totalWeight;
    public:
        MinimumSpanningTree();
        MinimumSpanningTree(int r, std::vector<int> &p, std::vector<int> &sO, double tW) : SpanningTree(r, p, sO) { totalWeight = tW; }
        ~MinimumSpanningTree() {}
        void printTree() const;
        double getTotalWeight() { return totalWeight; }
};

void MinimumSpanningTree::printTree() const {
    std::cout << "Root: " << root << std::endl;
    std::cout << "Edges: [";

    if (searchOrders.empty()) {
        std::cout << " ] (empty)" << std::endl;
        return;
    }

    for (size_t i = 0; i < searchOrders.size(); ++i) {
        int v = searchOrders[i];
        if (parent[v] != -1) {
            // Print an edge
            std::cout << " (" << parent[v] << ", " << v << ")";
            // Print a comma if this isn't the last edge
            if (i < searchOrders.size() - 1) std::cout << ",";
        }
    }
    
    std::cout << " ]" << std::endl;
    std::cout << "Total weight: " << totalWeight << std::endl;
}


#endif