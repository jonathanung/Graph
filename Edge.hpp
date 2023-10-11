#ifndef EDGE_HPP
#define EDGE_HPP

#include <iostream>

/**
 * @brief Edge Class
 * 
 * @author Jonathan Ungs
 */
class Edge {
    public:
        int p1;
        int p2;
        Edge(int a, int b) { p1 = a, p2 = b; }
        friend std::ostream& operator<<(std::ostream &, const Edge &);
};
std::ostream& operator<<(std::ostream & o, const Edge & e) {
    o << "{" << e.p1 << ", " << e.p2 << "}";
    return o;
}

#endif