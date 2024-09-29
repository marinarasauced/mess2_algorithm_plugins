#ifndef MESS2_ALGORITHM_PLUGINS_GRAPH_HPP
#define MESS2_ALGORITHM_PLUGINS_GRAPH_HPP

#include "mess2_algorithm_plugins/common.hpp"

namespace mess2_algorithms
{
    /**
     * @brief defines a vertex in a graph.
     */
    struct graph_vertex {
        double x;
        double y;
        double theta;
    };

    /**
     * @brief defines an edge in a graph.
     */
    struct graph_edge {
        int64_t index_parent;
        int64_t index_child;
        std::string type;
    };

    /**
     * @brief aliases of vectors of vertices and edges.
     */
    using graph_vertices = std::vector<graph_vertex>;
    using graph_edges = std::vector<graph_edge>;

    /**
     * @brief an operator for the graph map.
     */
    struct graph_hash {
        template <typename T1, typename T2>
        std::size_t operator()(const std::pair<T1, T2>& p) const {
            auto hash1 = std::hash<T1>{}(p.first);
            auto hash2 = std::hash<T2>{}(p.second);
            return hash1 ^ (hash2 << 1);
        }
    };

    /**
     * @brief aliases for a map of vertices to point2d on graph.
     */
    using graph_map = std::unordered_map<std::pair<double, double>, std::vector<int64_t>, graph_hash>;

    /**
     * @class Graph
     * @brief manages the creation and retrieval of graph elements for a low level search.
     * 
     * this class provides the tools for creating and managing a graph of vertices and edges.
     */
    class Graph
    {
    public:
        /**
         * @brief constructs a class instance of Graph
         * 
         * this constructor fills the vertices and edges of the instance using a domain and range with a resolution of the number of points in the domain and range.
         * 
         * @param x_mesh an arma matrix of x values in graph.
         * @param y_mesh an arma matrix of y values in graph.
         * @param resolution the number of elements in domain or range.
         * @param use_diagonals a bool to determine if diagonals should be generated.
         */
        Graph(const arma::mat& x_mesh, const arma::mat& y_mesh, const int64_t& resolution, const bool& use_diagonals);

        /**
         * @brief generates vertices.
         * 
         * this method generates vertices for each (x, y) coordinate pair at 45 degree incremements if diagonals are used and at 90 degree incremements if diagonals are not used.
         * 
         * @param x_mesh an arma matrix of x values in graph.
         * @param y_mesh an arma matrix of y values in graph.
         * @param use_diagonals a bool to determine if diagonals should be generated.
         */
        void generate_vertices(const arma::mat& x_mesh, const arma::mat& y_mesh, const bool& use_diagonals);

        /**
         * @brief generates a map of vertices to (x, y) coordinate pairs.
         * 
         * this method maps vertices to their coordinates for future computations since there may be N vertices per (x, y) coordinate pair.
         * 
         * @param resolution number of vertices in domain and range dimensions.
         */
        void generate_map(const int64_t& resolution);

        /**
         * @brief generates edges.
         * 
         * this method generates edges with the assumption that if an actor waits then it may wait, rotate, or translate (if valid), if an actor rotates then it must translate, if an actor translates then it may wait, rotate, or translate (if valid).
         * 
         * @param x_mesh an arma matrix of x values in graph.
         * @param y_mesh an arma matrix of y values in graph.
         * @param use_diagonals a bool to determine if diagonals should be generated.
         */
        void generate_edges(const arma::mat& x_mesh, const arma::mat& y_mesh, const bool& use_diagonals);

        /**
         * @brief generate adjacencies.
         * 
         * this method generates adjacencies such that for each vertex, the indices of all valid edges are stored in a vector at that vertices index.
         */
        void generate_adjacencies();

        /**
         * @brief looks up a vertex.
         * 
         * this method looks up the vertex of an index.
         * 
         * @param index_vertex the index of the vertex to lookup.
         * @return a graph vertex.
         */
        graph_vertex lookup_vertex(const int64_t& index_vertex) const;

        /**
         * @brief looks up indices of a map entry.
         * 
         * this method looks up the indices of vertices at an (x, y) coordinate.
         * 
         * @param x the x coordinate.
         * @param y the y coordinate.
         * @return a vector of indices of vertices at (x, y) coordinate.
         */
        std::vector<int64_t> lookup_map(const double& x, const double& y) const;

        /**
         * @brief looks up an edge.
         * 
         * this method looks up the edge of an index.
         * 
         * @param index_edge the index of the edge to lookup.
         * @return a graph edge.
         */
        graph_edge lookup_edge(const int64_t& index_edge) const;

        /**
         * @brief looks up valid indices of edges adjacent to current vertex.
         * 
         * this method looks up the valid indices that an actor may transition across from a given vertex.
         * 
         * @param index_vertex the index of the vertex at which adjacencies are to be looked up.
         * @param type the type of previous edge transition.
         * @return a vector of indices of valid edges.
         */
        std::vector<int64_t> lookup_adjacencies(const int64_t& index_vertex, const std::string& type) const;

        /**
         * @brief looks up the index of an edge using the parent and child vertex indicies.
         */
        int64_t lookup_index_edge(const int64_t& index_parent, const int64_t& index_child) const;

        /**
         * @brief looks up the index of a vertex using the x, y, and theta values.
         */
        int64_t lookup_index_vertex(const double& x, const double& y, const double& theta) const;

        /**
         * @brief prints current info about the class.
         */
        void info();

        int64_t n_vertices;
        int64_t n_edges;

        /**
         * 
         */
        void save_edges(const std::string& path_edges) const;

        /**
         * 
         */
        void save_vertices(const std::string& path_vertices) const;

    private:
        graph_vertices vertices_;
        graph_map map_;
        graph_edges edges_;

        std::vector<std::vector<int64_t>> adjacencies_wait_;
        std::vector<std::vector<int64_t>> adjacencies_rotate_;
        std::vector<std::vector<int64_t>> adjacencies_translate_;
    };

    /**
     * @brief generate a mesh consisting of two NxN matrices for x and y values respectively.
     * 
     * this function generates two NxN matrices comprising a mesh of x and y values on the specified domain and range with the specified resolution N.
     * 
     * @param x_min the minimum x value in graph domain.
     * @param x_max the maximum x value in graph domain.
     * @param y_min the minimum y value in graph range.
     * @param y_max the maximum y value in graph range.
     * @param resolution the number of vertices in domain and range dimensions.
     * @return a tuple of the x and y meshes.
     */
    std::tuple<arma::mat, arma::mat> generate_mesh(const double& x_min, const double& x_max, const double& y_min, const double& y_max, const int64_t& resolution);

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_GRAPH_HPP