#include <iostream>
#include <string>
#include <limits>
#include "WeightedGraph.hpp" // Assuming this header defines WeightedGraph and WeightedEdge properly

int main() {
    // Test case 1: Graph with a negative cycle
    {
        std::vector<std::string> vertices = {"A", "B", "C", "D"};
        std::vector<WeightedEdge> edges = {
            WeightedEdge(0, 1, 1),   // A -> B (weight 1)
            WeightedEdge(1, 2, -1),  // B -> C (weight -1)
            WeightedEdge(2, 3, -1),  // C -> D (weight -1)
            WeightedEdge(3, 1, -1),  // D -> B (weight -1), creates a negative cycle
        };

        WeightedGraph<std::string> graph(vertices, edges);
        try {
            ShortestPathTree distances = graph.bellmanFordShortestPath(0); // Starting from vertex A
            std::cout << "Test 1 (No negative cycle): Shortest distances from A to each vertex Bellman-Ford:" << std::endl;
            distances.printTree();
        } catch (std::invalid_argument& e) {
            std::cout << "Test 1 (Negative cycle): " << e.what() << std::endl;
        }
        try {
            ShortestPathTree distances = graph.dijkstraShortestPath(0); // Starting from vertex A
            std::cout << "Test 1 (Not negative graph): Shortest distances from A to each vertex Dijkstra's:" << std::endl;
            distances.printTree();
        } catch (std::invalid_argument& e) {
            std::cout << "Test 1 (Negative graph): " << e.what() << std::endl;
        }
    }

    // Test case 2: Graph without a negative cycle
    {
        std::vector<std::string> vertices = {"A", "B", "C", "D", "E"};
        std::vector<WeightedEdge> edges = {
            WeightedEdge(0, 1, 4),   // A -> B (weight 4)
            WeightedEdge(0, 2, 2),   // A -> C (weight 2)
            WeightedEdge(1, 2, 3),   // B -> C (weight 3)
            WeightedEdge(1, 3, 2),   // B -> D (weight 2)
            WeightedEdge(1, 4, 3),   // B -> E (weight 3)
            WeightedEdge(2, 1, 1),   // C -> B (weight 1)
            WeightedEdge(2, 3, 4),   // C -> D (weight 4)
            WeightedEdge(2, 4, 5),   // C -> E (weight 5)
            WeightedEdge(3, 4, 1),   // D -> E (weight 1)
        };

        WeightedGraph<std::string> graph(vertices, edges);
        try {
            ShortestPathTree distances = graph.bellmanFordShortestPath(0); // Starting from vertex A
            std::cout << "Test 2 (No negative cycle): Shortest distances from A to each vertex Bellman-Ford:" << std::endl;
            distances.printTree();
        } catch (std::invalid_argument& e) {
            std::cout << "Test 2 (Negative cycle detected with Bellman-Ford): " << e.what() << std::endl;
        }
        try {
            ShortestPathTree distances = graph.dijkstraShortestPath(0); // Starting from vertex A
            std::cout << "Test 2 (Not negative graph): Shortest distances from A to each vertex Dijkstra's:" << std::endl;
            distances.printTree();
        } catch (std::invalid_argument& e) {
            std::cout << "Test 2 (Negative graph detected with Dijkstra's): " << e.what() << std::endl;
        }
    }

    // Test case 3: Graph with 10 vertices, negative edges, but no negative cycles
    {
        std::vector<std::string> vertices = {"A", "B", "C", "D", "E", "F", "G", "H", "I", "J"};
        std::vector<WeightedEdge> edges = {
            WeightedEdge(0, 1, 6),    // A -> B (weight 6)
            WeightedEdge(0, 2, 5),    // A -> C (weight 5)
            WeightedEdge(1, 3, -2),   // B -> D (weight -2)
            WeightedEdge(2, 1, -2),   // C -> B (weight -2)
            WeightedEdge(2, 4, 1),    // C -> E (weight 1)
            WeightedEdge(3, 5, 3),    // D -> F (weight 3)
            WeightedEdge(4, 3, -2),   // E -> D (weight -2)
            WeightedEdge(4, 5, 4),    // E -> F (weight 4)
            WeightedEdge(5, 7, 2),    // F -> H (weight 2)
            WeightedEdge(5, 6, 2),    // F -> G (weight 2)
            WeightedEdge(7, 8, -3),   // H -> I (weight -3)
            WeightedEdge(6, 9, 1),    // G -> J (weight 1)
            WeightedEdge(8, 9, 3),    // I -> J (weight 3)
        };

        WeightedGraph<std::string> graph(vertices, edges);
        try {
            ShortestPathTree distances = graph.bellmanFordShortestPath(0); // Starting from vertex A
            std::cout << "Test 3 (Negative edges, no negative cycle): Shortest distances from A to each vertex Bellman-Ford:" << std::endl;
            distances.printTree();
        } catch (std::invalid_argument& e) {
            std::cout << "Test 3 (Negative cycle detected with Bellman-Ford): " << e.what() << std::endl;
        }
        try {
            ShortestPathTree distances = graph.dijkstraShortestPath(0); // Starting from vertex A
            std::cout << "Test 3 (Not negative graph): Shortest distances from A to each vertex Dijkstra's:" << std::endl;
            distances.printTree();
        } catch (std::invalid_argument& e) {
            std::cout << "Test 3 (Negative graph detected with Dijkstra's): " << e.what() << std::endl;
        }
    }

    //Prims test 1
    {
        std::vector<std::string> vertices = {"A", "B", "C", "D", "E", "F", "G", "H", "I", "J"};
        std::vector<WeightedEdge> edges = {
            WeightedEdge(0, 1, 6),    // A -> B (weight 6)
            WeightedEdge(0, 2, 5),    // A -> C (weight 5)
            WeightedEdge(1, 3, -2),   // B -> D (weight -2)
            WeightedEdge(2, 1, -2),   // C -> B (weight -2)
            WeightedEdge(2, 4, 1),    // C -> E (weight 1)
            WeightedEdge(3, 5, 3),    // D -> F (weight 3)
            WeightedEdge(4, 3, -2),   // E -> D (weight -2)
            WeightedEdge(4, 5, 4),    // E -> F (weight 4)
            WeightedEdge(5, 7, 2),    // F -> H (weight 2)
            WeightedEdge(5, 6, 2),    // F -> G (weight 2)
            WeightedEdge(7, 8, -3),   // H -> I (weight -3)
            WeightedEdge(6, 9, 1),    // G -> J (weight 1)
            WeightedEdge(8, 9, 3),    // I -> J (weight 3)
        };

        WeightedGraph<std::string> graph(vertices, edges, true);
        try{ 
            MinimumSpanningTree mst = graph.primMinimumSpanningTree(0); // Starting from vertex A
            std::cout << "Undirected graph:" << std::endl;
            mst.printTree();
        } catch (std::invalid_argument& e) {
            std::cout << "FAIL: " << e.what() << std::endl;
        }
    }


    return 0;
}

