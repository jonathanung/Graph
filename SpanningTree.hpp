#ifndef SPANNING_TREE_HPP
#define SPANNING_TREE_HPP

#include "Edge.hpp"
#include <vector>

/**
 * @brief Spanning Tree Class
 * 
 * @author Jonathan Ung
 */
class SpanningTree {
    protected:
        int root;
        std::vector<int> parent;
        std::vector<int> searchOrders;
    public:
        SpanningTree() {}
        SpanningTree(int, const std::vector<int> &, const std::vector<int> &);
        ~SpanningTree() {}
        int getRoot() const { return root; }
        std::vector<int> getSearchOrders() const { return std::vector<int>(searchOrders); }
        int getParent(int i) const { return parent[i]; }
        size_t getNumberOfVerticesFound() const { return searchOrders.size(); }
        std::vector<int> getPath(int) const;

        void printTree() const;
};

SpanningTree::SpanningTree(int r, const std::vector<int>& p, const std::vector<int>& sO) {
    root = r;
    parent = p;
    searchOrders = sO;
}

std::vector<int> SpanningTree::getPath(int v) const {
    std::vector<int> path;
    do {
        path.push_back(v);
        v = parent[v];
    } while (v >= 0);
    return path;
}

void SpanningTree::printTree() const {
    std::cout << "Root: " << root << std::endl;
    std::cout << "Edges: [ ";
    for (int v : searchOrders) {
        if (parent[v] != -1) {
            std::cout << Edge(parent[v], v);
            if (v != searchOrders[searchOrders.size()-1]) std::cout << ", ";
        }
    }
    std::cout << " ]" << std::endl;
}

#endif