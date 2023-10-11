#ifndef WEIGHTED_GRAPH_HPP
#define WEIGHTED_GRAPH_HPP

#include "Graph.hpp"
#include "WeightedEdge.hpp"
#include "SpanningTree.hpp"
#include "ShortestPathTree.hpp"
#include "MinimumSpanningTree.hpp"

template <class V>
class WeightedGraph : public Graph<V>{
    protected:
        WeightedEdge *createEdge(int a, int b) { return new WeightedEdge(a, b); }
        WeightedEdge *createEdge(int a, int b, double c) { return new WeightedEdge(a, b, c); }
    public:
        WeightedGraph() : Graph<V>();
        WeightedGraph(std::vector<V> &, std::vector<WeightedEdge> &, bool = false);
        void printWeightedEdge() const;
        MinimumSpanningTree getMinimumSpanningTree() const;
        MinimumSpanningTree getMinimumSpanningTree(int) const;
        ShortestPathTree getShortestPath(int) const;
        bool addEdges(int, int, double);
};

#endif