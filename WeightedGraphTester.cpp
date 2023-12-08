#include <iostream>
#include <string>
#include "WeightedGraph.hpp"

int main() {
    std::vector<std::string> strs = {"0", "1", "2", "3", "4", "5", "6"};
    std::vector<WeightedEdge> edges = {
        WeightedEdge(0,1,2),
        WeightedEdge(1,2,6),
        WeightedEdge(2,3,8),
        WeightedEdge(3,4,3),
        WeightedEdge(4,5,2),
        WeightedEdge(5,0,1),
        WeightedEdge(6,4,4),
        WeightedEdge(6,3,6),
        WeightedEdge(6,1,1),
        WeightedEdge(0,6,3),
        WeightedEdge(5,6,5),
        WeightedEdge(2,6,1)
    };
    WeightedGraph<std::string> g = WeightedGraph<std::string>(strs, edges, false);
    g.printAdjacencyMatrix();
    // g.printWeightedEdges();
    // g.removeVertex("6");
    g.printEdges();
    // SpanningTree sp1 = g.depthFirstSearch(1);
    // sp1.printTree();
    // SpanningTree sp2 = g.breadthFirstSearch(1);
    // sp2.printTree();
    return 0;
}
