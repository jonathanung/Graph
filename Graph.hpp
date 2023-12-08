#ifndef GRAPH_HPP
#define GRAPH_HPP

#include "Edge.hpp"
#include "SpanningTree.hpp"
#include <iostream>
#include <vector>
#include <stack>
#include <queue>
#include <map>
#include <algorithm>
#include <stdexcept>

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
        std::vector<std::vector<bool>> adjacencyMatrix;
        size_t size;
        Edge *createEdge(int a, int b) { return new Edge(a, b); }

    public:
        Graph(); //WK9HW
        Graph(std::vector<V> &, int**, int, bool = false);
        Graph(std::vector<V> &, std::vector<Edge> &, bool = false);
        Graph(bool**, int); //WK9HW
        ~Graph() { clear(); }
        bool addVertex(V);
        bool addEdge(int, int);
        bool addEdge(int, int, bool);
        bool removeVertex(V);
        bool removeEdge(int, int);
        bool removeEdge(int, int, bool);
        size_t getSize() const { return size; }
        size_t getDegree(int i) const { return neighbors[i].size(); }
        V getVertex(int i) const { return vertices[i]; }
        std::vector<V> getVertices() const;
        std::vector<V> getNeighbors(int) const;
        void setAdjacencyMatrix();
        std::vector<std::vector<bool>> getAdjacencyMatrix() const { return adjacencyMatrix; }
        void printEdges() const; //WK9HW
        void printAdjacencyMatrix() const; //WK9HW
        void clear();
        size_t vertexCount() const { return size; } //WK9HW
        void printVertices() const; //WK9HW
        size_t edgeCount() const;   // WK9HW
        void showNeighbors(int) const; //WK9HW
        int outDegree(int a) const { return neighbors[a].size(); } //WK9HW
        int inDegree(int a) const; //WK9HW
        int maxDegree(int a) const;// WK9HW
        bool isDirected() const;      // WK9HW
        void printOutDegrees() const; //WK9HW
        void printInDegrees() const; //WK9HW
        void printMaxDegrees() const;// WK9HW
        bool hasEdge(int a, int b) const { return adjacencyMatrix[a][b] || adjacencyMatrix[b][a]; } // WK9HW
        int path(int, int) const; //WK9HW
        bool hasCycle() const; //WK9HW
        bool hasCycle(int, std::vector<bool> &, std::vector<bool> &) const; // WK9HW
        bool hasSelfLoop(int) const;
        void printCycles() const; // WK9HW
        void printCycle(int) const;

        SpanningTree depthFirstSearch(int) const;
        SpanningTree breadthFirstSearch(int) const;

        bool isBipartite() const;
        std::vector<V> topologicalSort() const;
        void topSortDFS(int, std::vector<bool> &, std::vector<V> &) const;
};

template <class V>
Graph<V>::Graph() {
    vertices = std::vector<V>();
    neighbors = std::vector<std::vector<Edge *>>();
    size = 0;
    adjacencyMatrix = std::vector<std::vector<bool>>();
}

template <class V>
Graph<V>::Graph(std::vector<V> &vert, int **edges, int numE, bool undirected) : Graph<V>() {
    for (V vertex : vert) {
        addVertex(vertex);
    }
    // size = vert.size();
    // setAdjacencyMatrix();
    for (size_t i = 0; i < numE; i++) addEdge(edges[i][0], edges[i][1], undirected);
}

template <class V>
Graph<V>::Graph(std::vector<V> & vert, std::vector<Edge> & edges, bool undirected) : Graph<V>() {
    for (V vertex : vert) {
        addVertex(vertex);
    }
    // size = vert.size();
    // setAdjacencyMatrix();
    for (Edge e : edges) addEdge(e.p1, e.p2, undirected);
}

template <>
Graph<int>::Graph(bool** aM, int n) : Graph<int>() {
    for (size_t i = 0; i < n; i++) {
        addVertex(i);
    }
    for (size_t i = 0; i < size; i++) {
        for (size_t j = 0; j < size; j++) {
            if (aM[i][j] == true) {
                addEdge(i, j, false);
            }
        }
    }
} //this is here because i know it will work with int

template <class V>
Graph<V>::Graph(bool** aM, int n) : Graph<V>() {
    for (size_t i = 0; i < n; i++) {
        addVertex(i);
    }
    for (size_t i = 0; i < size; i++) {
        for (size_t j = 0; j < size; j++) {
            if (aM[i][j] == true) {
                addEdge(i, j, false);
            }
        }
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
    setAdjacencyMatrix();
    return true;
}

template <class V>
bool Graph<V>::addEdge(int a, int b) {
    for(Edge* e: neighbors[a]) {
        if (e->p2 == b) return false;
    }
    neighbors[a].push_back(createEdge(a, b));
    adjacencyMatrix[a][b] = true;
    return true;
}

template <class V>
bool Graph<V>::addEdge(int a, int b, bool u) {
    bool c = addEdge(a, b);
    bool d = false;
    if (u) d = addEdge(b, a);
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
            setAdjacencyMatrix();
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
            adjacencyMatrix[a][b] = false;
            return true;
        }
    }
    return false;
}

template <class V>
bool Graph<V>::removeEdge(int a, int b, bool u) {
    bool c = removeEdge(a, b);
    bool d = false;
    if(u) d = removeEdge(b, a);
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
void Graph<V>::setAdjacencyMatrix() {
    if (size == 0) {
        adjacencyMatrix = std::vector<std::vector<bool>>();
        return;
    }
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
    adjacencyMatrix = mat;
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
        std::cout << "[  ";
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
    std::vector<int> searchOrders = std::vector<int>();
    std::vector<int> parent = std::vector<int>(size, -1);
    std::vector<bool> visited = std::vector<bool>(size, false);
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
    std::vector<int> searchOrders = std::vector<int>();
    std::vector<int> parent = std::vector<int>(size, -1);
    std::vector<bool> visited = std::vector<bool>(size, false);
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

template <class V>
void Graph<V>::printVertices() const {
    for (size_t i = 0; i < size; i++) {
        std::cout << "Index " << i << " holding Vertex " << vertices[i];
        std::cout << std::endl;
    }
}

template <class V>
size_t Graph<V>::edgeCount() const {
    size_t count = 0;
    for (size_t i = 0; i < size; i++) {
        count += neighbors[i].size();
    }
    return count;
}

template <class V>
void Graph<V>::showNeighbors(int a) const {
    std::cout << "Neighbors of vertex " << getVertex(a) << ": ";
    for (size_t i = 0; i < size; i++){
        if (adjacencyMatrix[i][a] == true || adjacencyMatrix[a][i] == true) {
            std::cout << i << " ";
        }
    }
    std::cout << std::endl;
}

template <class V>
bool Graph<V>::isDirected() const {
    for (size_t i = 0; i < size; i++) {
        for (size_t j = 0; j < size - i; j++) {
            if (adjacencyMatrix[i][j] != adjacencyMatrix[j][i]) {
                return true;
            }
        }
    }
    return false;
}

template <class V>
int Graph<V>::inDegree(int a) const {
    int degree = 0;
    for (size_t i = 0; i < size; i++) {
        if (adjacencyMatrix[i][a] == true) {
            degree++;
        }
    }
    return degree;
}

template <class V>
int Graph<V>::maxDegree(int a) const {
    int degree = 0;
    for (size_t i = 0; i < size; i++)
    {
        if (adjacencyMatrix[i][a] || adjacencyMatrix[a][i]) {
            degree++;
            if (a == i) {
                degree++;
            }
        }
    }
    return degree;
}

template <class V>
void Graph<V>::printOutDegrees() const {
    for (size_t i = 0; i < size; i++) {
        std::cout << "Index " << i << " holding Vertex " << vertices[i] << " of degree " << outDegree(i);
        std::cout << std::endl;
    }
}

template <class V>
void Graph<V>::printInDegrees() const {
    for (size_t i = 0; i < size; i++) {
        std::cout << "Index " << i << " holding Vertex " << vertices[i] << " of degree " << inDegree(i);
        std::cout << std::endl;
    }
}

template <class V>
void Graph<V>::printMaxDegrees() const {
    for (size_t i = 0; i < size; i++) {
        std::cout << "Index " << i << " holding Vertex " << vertices[i] << " of degree " << maxDegree(i);
        std::cout << std::endl;
    }
}

template <class V>
int Graph<V>::path(int a, int b) const {
    if (a == b)
        return 0;
    if (a < 0 || a >= size || b < 0 || b >= size)
        return -1;
    std::vector<bool> visited = std::vector<bool>(size, false);
    std::queue<int> q = std::queue<int>();
    visited[a] = true;
    q.push(a);
    for (int i = 0; i < size && !q.empty(); i++) {
        int levelSize = q.size();
        for (int j = 0; j < levelSize; j++) {
            int curr = q.front();
            q.pop();
            for (Edge* e : neighbors[curr]) {
                int adjVertex = (e->p1 == curr) ? e->p2 : e->p1;
                if (adjVertex == b) return i + 1;
                if (!visited[adjVertex]) {
                    visited[adjVertex] = true;
                    q.push(adjVertex);
                }
            }
        }
    }
    return -1;
}

template <class V>
bool Graph<V>::hasCycle() const {
    std::vector<bool> visited(size, false);
    std::vector<bool> recStack(size, false);

    for (int i = 0; i < size; i++) {
        if (!visited[i]) {
            if (hasCycle(i, visited, recStack)) {
                return true;
            }
        }
    }
    return false;
}

template <class V>
bool Graph<V>::hasCycle(int v, std::vector<bool> &visited, std::vector<bool> &recStack) const {
    if (!visited[v]) {
        visited[v] = true;
        recStack[v] = true;

        for (Edge *edge : neighbors[v]) {
            int next = edge->p2;
            if (!visited[next] && hasCycle(next, visited, recStack)) {
                return true;
            } else if (recStack[next]) {
                return true;
            }
        }
    }
    recStack[v] = false;  // remove the vertex from recursion stack
    return false;
}

template <class V>
void Graph<V>::printCycles() const {
    for (int i = 0; i < size; i++) {
        printCycle(i);
    }
}

template <class V>
bool Graph<V>::hasSelfLoop(int i) const {
    return adjacencyMatrix[i][i];
}

template <class V>
void Graph<V>::printCycle(int v) const {
    std::vector<int> searchOrders = std::vector<int>();
    searchOrders.push_back(v);
    std::vector<int> parentOf = std::vector<int>(size, -1);
    std::vector<bool> visited = std::vector<bool>(size, false);
    visited[v] = true;
    std::stack<int> next = std::stack<int>();
    next.push(v);
    while (!next.empty()) {
        bool seen = false;
        for (Edge *edge : neighbors[next.top()]) {
            if (!visited[edge->p2]) {
                visited[edge->p2] = true;
                parentOf[edge->p2] = next.top();
                next.push(edge->p2);
                searchOrders.push_back(edge->p2);
                seen = true;
                break;
            }
            else if (edge->p2 == v && parentOf[next.top()] != v) {
                std::cout << "Cycle at vertex " << vertices[v] << " at index " << v << ": {";
                for (int u : searchOrders) {
                    std::cout << u << " -> ";
                }
                std::cout << v << "}" << std::endl;
                return;
            }
        }
        if (!seen) {
            next.pop();
            searchOrders.pop_back();
        }
    }
}

template<class V>
bool Graph<V>::isBipartite() const {
    std::unordered_map<int, int> colors;
    for (int i = 0; i < size; i++) {
        if (colors.find(i) == colors.end()) {
            std::queue<int> q;
            q.push(i);
            colors[i] = 0;
            while (!q.empty()) {
                int u = q.front();
                for (Edge* e : neighbors[u]) {
                    int v = e->p2;
                    if (colors.find(v) == colors.end()) {
                        colors[v] = 1 - colors[u];
                        q.push(v);
                    } else if (colors[v] == colors[u]) {
                        return false;
                    }
                }
                q.pop();
            }
        }
    }
    return true;
}

template<class V>
std::vector<V> Graph<V>::topologicalSort() const {
    if (!isDirected() || hasCycle()) {
        throw std::invalid_argument("Graph may not be undirected or have a cycle in order for topological sort to be valid.");
    }
    std::vector<int> sources;
    for (int i = 0; i < size; i++) {
        if (inDegree(i) == 0) {
            sources.push_back(i);
        }
    }
    if (sources.size() == 0) {
        throw std::invalid_argument("There must be a source for topological sort to function.");
    }
    std::vector<V> res;
    std::vector<bool> visited(size, false);
    for (int u : sources) {
        topSortDFS(u, visited, res);
    }
    std::reverse(res.begin(), res.end());
    return res;
}

template<class V>
void Graph<V>::topSortDFS(int u, std::vector<bool> & visited, std::vector<V> & res) const{
    visited[u] = true;
    for (Edge* e : neighbors[u]) {
        int v = e->p2;
        if (!visited[v]) {
            topSortDFS(v, visited, res);
        }
    }
    res.push_back(vertices[u]);
}

#endif

