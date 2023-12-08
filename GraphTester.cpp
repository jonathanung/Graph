#include <iostream>
#include <string>
#include "Graph.hpp"

int main() {
    std::vector<std::string> strs = {"0", "1", "2", "3", "4", "5", "6"};
    std::vector<Edge> edges = {
        Edge(0,1),
        Edge(1,2),
        Edge(2,3),
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
    g.removeVertex("6");
    g.printEdges();
    SpanningTree sp1 = g.depthFirstSearch(1);
    sp1.printTree();
    SpanningTree sp2 = g.breadthFirstSearch(1);
    sp2.printTree();

        // Create vertices
    std::vector<std::string> strs2 = {"A", "B", "C", "D", "E", "F"};

    // Create edges for a DAG
    std::vector<Edge> edges2 = {
        Edge(0, 1), // A -> B
        Edge(0, 2), // A -> C
        Edge(1, 3), // B -> D
        Edge(2, 3), // C -> D
        Edge(3, 4), // D -> E
        Edge(2, 5), // C -> F
    };

    // Create a graph with the vertices and edges
    Graph<std::string> graph(strs2, edges2, false); // true for directed graph

    // Perform topological sort
    try {
        std::vector<std::string> sorted = graph.topologicalSort();
        std::cout << "Topological Sort: ";
        for (const std::string& v : sorted) {
            std::cout << v << " ";
        }
        std::cout << std::endl;
    } catch (const std::invalid_argument& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    return 0;
}
