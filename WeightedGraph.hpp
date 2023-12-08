#ifndef WEIGHTED_GRAPH_HPP
#define WEIGHTED_GRAPH_HPP

#include "Graph.hpp"
#include "WeightedEdge.hpp"
#include "SpanningTree.hpp"
#include "ShortestPathTree.hpp"
#include "MinimumSpanningTree.hpp"
#include <iostream>
#include <vector>
#include <stack>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include "PathTracer.hpp"

template <class V>
/**
 * @brief Weighted Graph class
 * 
 * @author Jonathan Ung
 */
class WeightedGraph : public Graph<V>{
    protected:
        WeightedEdge *createEdge(int a, int b) { return new WeightedEdge(a, b); }
        WeightedEdge *createEdge(int a, int b, double c) { return new WeightedEdge(a, b, c); }
        //generally a weighted graph would have another implementation of an adjacency matrix too, but i
        // am frankly too lazy to implement this ! :)
        /* might look like:
        [∞, 2, 3]
        [2, ∞, ∞]
        [3, ∞, ∞]
        where ∞/MAX_INT signifies no edge between two vertices. */
    public:
        WeightedGraph() : Graph<V>() {}
        WeightedGraph(std::vector<V> &, std::vector<WeightedEdge> &, bool = false);
        ~WeightedGraph() {}
        bool addEdge(int, int, double);
        bool addEdge(int, int, double, bool);
        bool updateWeight(int, int, double);
        bool updateWeight(int, int, double, bool);
        void printEdges() const;
        bool isNegative() const;
        MinimumSpanningTree primMinimumSpanningTree(int) const; // TODO
        ShortestPathTree dijkstraShortestPath(int) const; //TODO
        ShortestPathTree bellmanFordShortestPath(int) const; //TODO
        /* other algorithms that could be implemented:
        ShortestPathTree floydWarshallShortestPath(int) const;
        MinimumSpanningTree kruskalMinimumSpanningTree(int) const;
        */
};

template <class V>
WeightedGraph<V>::WeightedGraph(std::vector<V> & vert, std::vector<WeightedEdge> & edges, bool undirected) {
    for (V vertex : vert) {
        this->addVertex(vertex);
    }
    this->size = vert.size();
    for (WeightedEdge e : edges) addEdge(e.p1, e.p2, e.weight, undirected);
}

template <class V>
bool WeightedGraph<V>::addEdge(int a, int b, double w) {
    for(Edge* ed: this->neighbors[a]) {
        WeightedEdge* e = static_cast<WeightedEdge*>(ed);
        if (e->p2 == b) return false;
    }
    this->neighbors[a].push_back(createEdge(a, b, w));
    return true;
}

template <class V>
bool WeightedGraph<V>::addEdge(int a, int b, double w, bool u) {
    bool c = addEdge(a, b, w);
    bool d = false;
    if (u) d = addEdge(b, a, w);
    return c || d;
}

template <class V>
bool WeightedGraph<V>::updateWeight(int a, int b, double w) {
    for(Edge* ed: this->neighbors[a]) {
        WeightedEdge* e = static_cast<WeightedEdge*>(ed);
        if (e->p2 == b){
            e->weight = w;
            return true;
        }
    }
    return false;
}

template <class V>
bool WeightedGraph<V>::updateWeight(int a, int b, double w, bool u) {
    bool c = updateWeight(a, b, w);
    bool d = false;
    if (u) d = updateWeight(b, a, w);
    return c || d;
}

template <class V>
void WeightedGraph<V>::printEdges() const {
    for (size_t i = 0; i < this->size; i++) {
        std::cout << "Index " << i << " holding Vertex " << this->vertices[i] << ": ";
        for (size_t j = 0; j < this->neighbors[i].size(); j++) {
            std::cout << *(static_cast<WeightedEdge*>((this->neighbors[i][j])));
            if (j < this->neighbors[i].size() - 1) std::cout << ", ";
        }
        std::cout << std::endl;
    }
}

template <class V>
bool WeightedGraph<V>::isNegative() const {
    for (std::vector<Edge*> vec : this->neighbors) {
        for (Edge* edge : vec) {
            if (static_cast<WeightedEdge*>(edge)->weight < 0) {
                return true;
            }
        }
    }
    return false;
}

template<class V>
MinimumSpanningTree WeightedGraph<V>::primMinimumSpanningTree(int source) const {

}

template<class V>
ShortestPathTree WeightedGraph<V>::dijkstraShortestPath(int source) const {
    if (this->isNegative()) {
        throw std::invalid_argument("You cannot do dijkstra's on a negative graph!");
    }
    std::vector<bool> visited(this->size, false);
    std::unordered_map<V, PathTracer<V>> history;
}

template<class V>
ShortestPathTree WeightedGraph<V>::bellmanFordShortestPath(int source) const {

}

#endif