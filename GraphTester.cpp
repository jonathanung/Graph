#include <iostream>
#include <string>
#include "Graph.hpp"

int main() {
    std::vector<std::string> strs = {"0", "1", "2", "3", "4", "5", "6"};
    std::vector<Edge> edges = {
        Edge(0, 1),
        Edge(1, 2),
        Edge(2, 3),
        Edge(3,4),
        Edge(4,5),
        Edge(5,0),
        Edge(6,4),
        Edge(6,3),
        Edge(6,1)
    };
    Graph<std::string> g = Graph<std::string>(strs, edges, true);
    g.printAdjacencyMatrix();
    // g.printEdges();
    // g.removeVertex("6");
    g.printEdges();
    SpanningTree sp1 = g.depthFirstSearch(1);
    sp1.printTree();
    SpanningTree sp2 = g.breadthFirstSearch(1);
    sp2.printTree();
    return 0;
}
