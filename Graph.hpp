#ifndef GRAPH_HPP
#define GRAPH_HPP

#include "Edge.hpp"
#include "SpanningTree.hpp"
#include <iostream>
#include <vector>
#include <stack>
#include <queue>

template <class V>
/**
 * @brief Graph class
 * 
 * @author Jonathan Ung
 */
class Graph {
    protected:
        std::vector<V> vertices;
        std::vector<std::vector<Edge *>> neighbors;
        size_t size;
        Edge *createEdge(int a, int b) { return new Edge(a, b); }

    public:
        Graph();
        Graph(std::vector<V> &, int**, int, bool = false);
        Graph(std::vector<V> &, std::vector<Edge> &, bool = false);
        ~Graph() { clear(); }
        bool addVertex(V);
        bool addEdge(int, int);
        bool addUndirectedEdge(int, int);
        bool removeVertex(V);
        bool removeEdge(int, int);
        bool removeUndirectedEdge(int, int);
        size_t getSize() const { return size; }
        size_t getDegree(int i) const { return neighbors[i].size(); }
        V getVertex(int i) const { return vertices[i]; }
        std::vector<V> getVertices() const;
        std::vector<V> getNeighbors(int) const;
        std::vector<std::vector<bool>> getAdjacencyMatrix() const;
        void printEdges() const;
        void printAdjacencyMatrix() const;
        void clear();

        SpanningTree depthFirstSearch(int) const;
        SpanningTree breadthFirstSearch(int) const;
};

template <class V>
Graph<V>::Graph() {
    vertices = std::vector<V>();
    neighbors = std::vector<std::vector<Edge *>>();
    size = 0;
}

template <class V>
Graph<V>::Graph(std::vector<V> &vert, int **edges, int numE, bool undirected) : Graph<V>() {
    for (V vertex : vert) {
        addVertex(vertex);
    }
    size = vert.size();
    if (!undirected) {
        for (size_t i = 0; i < numE; i++) addEdge(edges[i][0], edges[i][1]);
    }
    else {
        for (size_t i = 0; i < numE; i++) addUndirectedEdge(edges[i][0], edges[i][1]);
    }
}

template <class V>
Graph<V>::Graph(std::vector<V> & vert, std::vector<Edge> & edges, bool undirected) : Graph<V>() {
    for (V vertex : vert) {
        addVertex(vertex);
    }
    size = vert.size();
    if (!undirected) {
        for (Edge e : edges) addEdge(e.p1, e.p2);
    }
    else {
        for (Edge e : edges) addUndirectedEdge(e.p1, e.p2);
    }
}

template <class V>
bool Graph<V>::addVertex(V vertex) {
    for(V v : vertices) {
        if(v == vertex) return false;
    }
    vertices.push_back(vertex);
    size++;
    neighbors.resize(size);
    addEdge(size - 1, size - 1);
    return true;
}

template <class V>
bool Graph<V>::addEdge(int a, int b) {
    for(Edge* e: neighbors[a]) {
        if (e->p2 == b) return false;
    }
    neighbors[a].push_back(createEdge(a, b));
    return true;
}

template <class V>
bool Graph<V>::addUndirectedEdge(int a, int b) {
    bool c = addEdge(a, b);
    bool d = addEdge(b, a);
    return c || d;
}

template <class V>
bool Graph<V>::removeVertex(V vertex) {
    for (size_t i = 0; i < vertices.size(); i++) {
        if (vertices[i] == vertex) {
            for (Edge* e: neighbors[i]) delete e;
            neighbors[i].clear();
            for (size_t j = i; j < vertices.size() -1; j++) {
                vertices[j] = vertices[j+1];
                neighbors[j] = neighbors[j+1];
            }
            vertices.pop_back();
            neighbors.pop_back();
            for (std::vector<Edge*> edges : neighbors) {
                for (Edge* e : edges) {
                    if (e->p2 == i) removeEdge(e->p1, e->p2);
                }
            }
            size--;
            return true;
        }
    }
    return false;
}

template <class V>
bool Graph<V>::removeEdge(int a, int b) {
    for (size_t i = 0; i < neighbors[a].size(); i++){
        if (neighbors[a][i]->p2 == b) {
            delete neighbors[a][i];
            for (int j = i; j < neighbors[a].size()-1; j++) neighbors[a][j] = neighbors[a][j+1];
            neighbors[a].pop_back();
            return true;
        }
    }
    return false;
}

template <class V>
bool Graph<V>::removeUndirectedEdge(int a, int b) {
    bool c = removeEdge(a, b);
    bool d = removeEdge(b, a);
    return c || d;
}

template <class V>
std::vector<V> Graph<V>::getVertices() const {
    std::vector<V> cp = std::vector<V>(vertices);
    return cp;
}

template <class V>
std::vector<V> Graph<V>::getNeighbors(int i) const {
    std::vector<V> nb = std::vector<V>();
    for (Edge* e : neighbors[i]){
        nb.push_back(vertices[e->p2]);
    }
    return nb;
}

template <class V>
std::vector<std::vector<bool>> Graph<V>::getAdjacencyMatrix() const {
    std::vector<std::vector<bool>> mat = std::vector<std::vector<bool>>(size);
    for (size_t k = 0; k < size; k++) {
        for (size_t l = 0; l < size; l++) {
            mat[k].push_back(false);
        } 
    }
    for (size_t i = 0; i < size; i++) {
        for (size_t j = 0; j < neighbors[i].size(); j++) {
            Edge *e = neighbors[i][j];
            mat[e->p1][e->p2] = true;
        }
    }
    return mat;
}

template <class V>
void Graph<V>::printEdges() const {
    for (size_t i = 0; i < size; i++) {
        std::cout << "Index " << i << " holding Vertex " << vertices[i] << ": ";
        for (size_t j = 0; j < neighbors[i].size(); j++) {
            std::cout << *(neighbors[i][j]);
            if (j < neighbors[i].size() - 1) std::cout << ", ";
        }
        std::cout << std::endl;
    }
}

template <class V>
void Graph<V>::printAdjacencyMatrix() const {
    std::vector<std::vector<bool>> mat = getAdjacencyMatrix();
    for (size_t i = 0; i < mat.size(); i++) {
        std::cout << "Index " << i << " holding Vertex " << vertices[i] << ": " << "[  ";
        for (size_t j = 0; j < mat[i].size(); j++) {
            std::cout << mat[i][j] << " ";
        }
        std::cout << " ]" << std::endl;
    }
}

template <class V>
void Graph<V>::clear() {
    vertices.clear();
    for (std::vector<Edge *> vec : neighbors) {
        for (Edge* e : vec) {
            delete e;
        }
        vec.clear();
    }
    neighbors.clear();
    size = 0;
}

template <class V>
SpanningTree Graph<V>::depthFirstSearch(int v) const {
    std::vector<int> searchOrders;
    std::vector<int> parent(vertices.size());
    std::vector<bool> visited(vertices.size());
    for (size_t i = 0; i < vertices.size(); i++) {
        parent[i] = -1;
        visited[i] = false;
    }
    std::stack<int> next = std::stack<int>();
    next.push(v);
    visited[v] = true;
    searchOrders.push_back(v);
    while (!next.empty()) {
        bool seen = false;
        for (Edge* e : neighbors[next.top()]) {
            if(!visited[e->p2]) {
                visited[e->p2] = true;
                parent[e->p2] = next.top();
                next.push(e->p2);
                searchOrders.push_back(e->p2);
                seen = true;
                break;
            }
        }
        if (!seen) next.pop();
    }
    return SpanningTree(v, parent, searchOrders);
}

template <class V>
SpanningTree Graph<V>::breadthFirstSearch(int v) const {
    std::vector<int> searchOrders;
    std::vector<int> parent(vertices.size());
    std::vector<bool> visited(vertices.size());
    for (size_t i = 0; i < vertices.size(); i++) {
        parent[i] = -1;
        visited[i] = false;
    }
    std::queue<int> next = std::queue<int>();
    next.push(v);
    visited[v] = true;
    searchOrders.push_back(v);
    while (!next.empty()) {
        bool seen = false;
        for (Edge* e : neighbors[next.front()]) {
            if(!visited[e->p2]) {
                visited[e->p2] = true;
                parent[e->p2] = next.front();
                next.push(e->p2);
                searchOrders.push_back(e->p2);
                seen = true;
            }
        }
        if (!seen) next.pop();
    }
    return SpanningTree(v, parent, searchOrders);
}

#endif