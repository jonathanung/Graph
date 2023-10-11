#ifndef SPANNING_TREE_HPP
#define SPANNING_TREE_HPP

#include <vector>

/**
 * @brief Spanning Tree Class
 * 
 * @author Jonathan Ung
 */
class SpanningTree {
    private:
        int root;
        std::vector<int> parent;
        std::vector<int> searchOrders;
    public:
        SpanningTree();
        SpanningTree(int, std::vector<int> &, std::vector<int> &);
        SpanningTree(int, std::vector<int>);
        int getRoot() const { return root; }
        int getSearchOrders() const;
        int getParent(int i) const { return parent[i]; }
        size_t getNumberOfVerticesFound() const { return searchOrders.size(); }
        std::vector<int> getPath(int) const;

        void printTree() const;
};

#endif