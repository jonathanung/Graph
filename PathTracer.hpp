#include <vector>
template <class V>
/**
 * @brief PathTracer class
 * 
 * @author Jonathan Ung
 */
class PathTracer {
    public:
        int distance;
        std::vector<V> vertices;
        PathTracer();
        PathTracer(int, V);
        PathTracer(int, std::vector<V>);
        PathTracer(PathTracer &, int, V);
};

template <class V>
PathTracer<V>::PathTracer() {
    distance = -1;
    vertices = std::vector<V>();
}

template <class V>
PathTracer<V>::PathTracer(int d, V v) {
    distance = d;
    vertices = std::vector<V>();
    vertices.push_back(v);
} 

template <class V>
PathTracer<V>::PathTracer(int d, std::vector<V> v) {
    distance = d;
    vertices = v;
} 

template <class V>
PathTracer<V>::PathTracer(PathTracer& pT, int d, V v) {
    distance = pT.distance + d;
    vertices = std::vector<V>(pT.vertices);
    vertices.push_back(v);
} 