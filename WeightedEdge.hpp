#ifndef WEIGHTED_EDGE_HPP
#define WEIGHTED_EDGE_HPP

#include "Edge.hpp"
#include <iostream>

/**
 * @brief Weighted Edge Class
 * 
 * @author Jonathan Ung
 */
class WeightedEdge : public Edge {
    double weight;
    WeightedEdge(int a, int b) : Edge(a, b) { weight = 0; }
    WeightedEdge(int a, int b, double c) : Edge(a, b) { weight = c; }
    ~WeightedEdge() {}
    friend std::ostream& operator<<(std::ostream &, const WeightedEdge &);
};

std::ostream& operator<<(std::ostream & o, const WeightedEdge & e) {
    o << "{" << e.p1 << ", " << e.p2 << ", " << "[" << e.weight << "]" << "}";
    return o;
}

#endif