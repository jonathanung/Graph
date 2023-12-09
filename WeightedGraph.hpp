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
#include <limits>
#include <utility>
#include <stdexcept>
#include <numeric>

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
        std::vector<std::vector<double>> weightedAdjacencyMatrix;

    public:
        WeightedGraph() : Graph<V>() { weightedAdjacencyMatrix = std::vector<std::vector<double>>(); }
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
        bool relax(WeightedEdge *, std::vector<double> &, std::vector<int>& parent) const;
        /* other algorithms that could be implemented:
        ShortestPathTree floydWarshallShortestPath(int) const;
        MinimumSpanningTree kruskalMinimumSpanningTree(int) const;
        */
};

template <class V>
WeightedGraph<V>::WeightedGraph(std::vector<V> & vert, std::vector<WeightedEdge> & edges, bool undirected) : WeightedGraph<V>() {
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
    if (this->isDirected()) {
        throw std::invalid_argument("You cannot run prim's algorithm on a directed graph!");
    }
    std::vector<int> searchOrders;
    std::vector<int> parent(this->size, -1);
    std::vector<bool> visited(this->size, false);
    std::vector<double> distances(this->size, std::numeric_limits<double>::max());
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;
    //PQ sorts by first element in the pair
    distances[source] = 0;
    pq.push({0, source});
    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();
        if (visited[u]) {
            continue; 
        }

        visited[u] = true;
        searchOrders.push_back(u);
        for (Edge *e : this->neighbors[u]) {
            int v = e->p2;
            double weight = static_cast<WeightedEdge *>(e)->weight;
            if (!visited[v] && weight < distances[v]) { 
                parent[v] = u;
                distances[v] = weight;
                pq.push({distances[v], v}); 
            }
        }
    }
    double totalWeight = std::accumulate(distances.begin(), distances.end(), decltype(distances)::value_type(0.0));
    MinimumSpanningTree mST = MinimumSpanningTree(source, parent, searchOrders, totalWeight);
    return mST;
}

template<class V>
ShortestPathTree WeightedGraph<V>::dijkstraShortestPath(int source) const {
    if (this->isNegative()) {
        throw std::invalid_argument("You cannot use Dijkstra's algorithm on a graph with negative weights!");
    }
    std::vector<int> searchOrders;
    std::vector<int> parent(this->size, -1);
    std::vector<bool> visited(this->size, false);
    std::vector<double> distances(this->size, std::numeric_limits<double>::max());
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;
    //PQ sorts by first element in the pair
    distances[source] = 0;
    pq.push({0, source});
    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();
        if (visited[u]) {
            continue; 
        }

        visited[u] = true;
        searchOrders.push_back(u);
        for (Edge *e : this->neighbors[u]) {
            int v = e->p2;
            double weight = static_cast<WeightedEdge *>(e)->weight;
            if (!visited[v] && distances[u] + weight < distances[v]) { 
                parent[v] = u;
                distances[v] = distances[u] + weight;
                pq.push({distances[v], v}); 
            }
        }
    }
    ShortestPathTree sPT = ShortestPathTree(source, parent, searchOrders, distances);
    return sPT;
}

template<class V>
ShortestPathTree WeightedGraph<V>::bellmanFordShortestPath(int source) const {
    std::vector<int> searchOrders = std::vector<int>();
    std::vector<int> parent = std::vector<int>(this->size, -1);
    std::vector<double> distances = std::vector<double>(this->size, std::numeric_limits<double>::max());

    distances[source] = 0;
    int i = 1;
    for (; i < this->size; i++) {
        bool relaxed = false;
        for (int j = 0; j < this->size; j++) {
            if (distances[j] != std::numeric_limits<double>::max()) {
                for (Edge* e : this->neighbors[j]) {
                    relaxed = relax(static_cast<WeightedEdge*>(e), distances, parent) || relaxed;
                }
            }
        }
        if (!relaxed) break;
    }
    if (i == this->size) {
        for (int j = 0; j < this->size; j++) {
            if (distances[j] != std::numeric_limits<double>::max()) {
                for (Edge* e : this->neighbors[j]) {
                    if (relax(static_cast<WeightedEdge*>(e), distances, parent)) {
                        throw std::invalid_argument("Your graph has a negative cycle!");
                    }
                }
            }
        }
    }
    std::vector<std::vector<int>> parent_map(parent.size(), std::vector<int>());
    for (int i = 0; i < parent_map.size(); i++) {
        if (parent[i] == -1) continue;
        int j = parent[i];
        parent_map[j].push_back(i);
    }
    std::queue<int> q;
    q.push(source);
    std::vector<bool> pushed(parent.size(), false);
    pushed[source] = true;
    while(!q.empty()) {
        int u = q.front();
        for (int v : parent_map[u]) {
            if(!pushed[v]) {
                q.push(v);
                pushed[v] = true;
            }
        }
        searchOrders.push_back(u);
        q.pop();
    }
    ShortestPathTree sPT = ShortestPathTree(source, parent, searchOrders, distances);
    return sPT;
}

template<class V>
bool WeightedGraph<V>::relax(WeightedEdge * e, std::vector<double> & distances, std::vector<int>& parent) const {
    int u = e->p1;
    int v = e->p2;
    double weight = e->weight;
    if (distances[u] + weight < distances[v]) {
        distances[v] = distances[u] + weight;
        parent[v] = u;
        return true;
    }
    return false;
}

#endif