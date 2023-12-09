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
        void printTree() const;
        int getCost(int i) { return costs[i]; }
};

void ShortestPathTree::printTree() const {
    std::cout << "Root: " << root << std::endl;
    if (searchOrders.empty()) {
        std::cout << "The search orders are empty, no edges to display." << std::endl;
        return;
    }

    std::cout << "Paths:" << std::endl;
    for (size_t i = 0; i < searchOrders.size(); ++i) {
        int v = searchOrders[i];
        if (v != root) {  // Skip the root, as it has no path leading to it
            std::cout << "Path to " << v << " (cost: " << costs[v] << "): ";

            // Trace the path from vertex v to the root
            std::stack<int> path;
            for (int x = v; x != -1; x = parent[x]) {
                path.push(x);
            }

            // Print the path
            while (!path.empty()) {
                std::cout << path.top();
                path.pop();
                if (!path.empty()) {
                    std::cout << " -> ";
                }
            }
            std::cout << std::endl;
        }
    }
}



#endif