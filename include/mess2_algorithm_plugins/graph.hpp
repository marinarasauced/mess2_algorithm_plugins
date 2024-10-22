#ifndef MESS2_ALGORITHM_PLUGINS_GRAPH_HPP
#define MESS2_ALGORITHM_PLUGINS_GRAPH_HPP

#include "mess2_algorithm_plugins/common.hpp"

namespace mess2_algorithms
{
    /**
     * @brief defines the structure of a key of indices of a point in a three-dimensional mesh.
     * 
     * @param index_key the index of the key in keys.
     * @param i the row index of a point in a three-dimensional mesh.
     * @param j the column index of a point in a three-dimensional mesh.
     * @param k the plane index of a point in a three-dimensional mesh.
     */
    struct Key3D
    {
        int index_key; // the index of the key in keys.
        int i; // the row index of a point in a three-dimensional mesh.
        int j; // the column index of a point in a three-dimensional mesh.
        int k; // the plane index of a point in a three-dimensional mesh.

        bool operator == (const std::shared_ptr<Key3D> &_other) const {
            return (i == _other->i && j == _other->j && k == _other->k);
        }
    };


    /**
     * @brief defines the structure of a point in three-dimensional space.
     * 
     * @param index_point the index of the point in points.
     * @param key the shared Key3D pointer of the current Point3D.
     * @param x the x-coordinate of the point in three-dimensional space.
     * @param y the y-coordinate of the point in three-dimensional space.
     * @param z the z-coordinate of the point in three-dimensional space.
     * @param value_threat the static threat value of the point.
     * @param value_obstalce the static obstacle value of the point.
     */
    struct Point3D
    {
        int index_point; // the index of the point in points.
        std::shared_ptr<Key3D> key; // the shared Key3D pointer of the current Point3D.
        std::shared_ptr<double> x; // the x-coordinate of the point in three-dimensional space.
        std::shared_ptr<double> y; // the y-coordinate of the point in three-dimensional space.
        std::shared_ptr<double> z; // the z-coordinate of the point in three-dimensional space.
        std::shared_ptr<double> value_threat; // the static threat value of the point.
        std::shared_ptr<double> value_obstacle; // the static obstacle value of the point.
    };


    /**
     * @brief defines the structure of a vertex in a graph in three-dimensional space.
     * 
     * @param index_vertex the index of the vertex in vertices.
     * @param point the shared Point3D pointer where the vertex resides in three-dimensional space.
     * @param heading the heading of the vertex in three-dimensional space.
     */
    struct Vertex
    {
        int index_vertex; // the index of the vertex in vertices.
        std::shared_ptr<Point3D> point; // the shared Point3D pointer where the vertex resides in three-dimensional space.
        double heading; // the heading of the vertex in three-dimensional space.
    };


    /**
     * @brief defiens the types of edge transitions.
     * 
     *  - WAIT : the actor waits at the current point and heading.
     * 
     *  - ROTATE : the actor rotates at the current point to a new heading.
     * 
     *  - TRANSLATE_IN_PLANE : the actor translates from the current point to another point in plane.
     * 
     *  - TRANSLATE_OUT_OF_PLANE : the actor translates from the current point to another point out of plane.
     */
    enum edge_type { WAIT, ROTATE, TRANSLATE_IN_PLANE, TRANSLATE_OUT_OF_PLANE };


    /**
     * @brief defines the structure of an edge in a graph in three-dimensional space.
     * 
     * @param index_edge the index of the edge in edges.
     * @param vertex_parent the shared vertex pointer of the parent vertex.
     * @param vertex_child the shared vertex pointer of the child vertex.
     * @param type the type of the edge.
     */
    struct Edge
    {
        int index_edge; // the index of the edge in edges.
        std::shared_ptr<Vertex> vertex_parent; // the shared vertex pointer of the parent vertex.
        std::shared_ptr<Vertex> vertex_child; // the shared vertex pointer of the child vertex.
        edge_type type; // the type of the edge.
    };


    /**
     * @brief defines the structure of a linearly spaced vector of shared double pointers.
     */
    using Linspace1D = std::vector<std::shared_ptr<double>>;


    /**
     * @brief defines the structure of a matrix of linearly spaced vectors of shared double pointers.
     */
    using Mesh3D = std::vector<std::vector<Linspace1D>>;


    /**
     * @brief defines a graph of vertices and edges in three-dimensional space where vertices consist of a point and an in-plane heading.
     * 
     * @param _x a Linspace1D of x values of the graph.
     * @param _y a Linspace1D of y values of the graph.
     * @param _z a Linspace1D of z values of the graph.
     * @param _values_threat a Mesh3D of static threat values of the graph.
     * @param _values_obstacle a Mesh3D of static obstacle values of the graph.
     * @param _use_diagonals_in_plane a bool that determines whether in-plane diagonal edges are computed.
     * @param _threshold_theat an optional threshold that prevents an actor from transitioning to points with occupied threat exceeding the value.
     * @param _theshold_obstacle an optional threshold that determine the confidence whether a point contains an obstacle.
     */
    class Graph
    {
    public:
        int n_i; // the number of cols
        int n_j; // the numer of rows
        int n_k; // the number of planes

        int n_keys; // the number of shared Key3D pointers in keys
        int n_points; // the number of shared Point3D pointers in pointss
        int n_vertices; // the number of shared Vertex pointers in vertices
        int n_edges; // the number of shared Edge pointers in edges

        bool use_diagonals_in_plane = false; // determines whether diagonal edges will be generated in plane.
        double threshold_threat = MAX_COST; // determines the maximum allowable threat value in values_threat;
        double threshold_obstacles = 0.5; // determines whether or not a point contains an obstacle;

        double runtime_build; // the time to build the Graph instance.


        /**
         * @brief constructs a Graph instance without initializing attributes.
         */
        Graph() {};


        /**
         * @brief constructs a Graph instance with initialing attributes.
         */
        Graph(const Linspace1D &_x, const Linspace1D &_y, const Linspace1D &_z, const Mesh3D &_values_threat, const Mesh3D &_values_obstacles, const bool _use_diagonals_in_plane = false, const double &_threshold_threat = MAX_COST, const double &_threshold_obstacles = 0.5) : use_diagonals_in_plane(_use_diagonals_in_plane), threshold_threat(_threshold_threat), threshold_obstacles(_threshold_obstacles), values_threat(_values_threat), values_obstacles(_values_obstacles), values_x(_x), values_y(_y), values_z(_z)
        {
            auto tic = std::clock();

            auto n_x = values_x.size();
            auto n_y = values_y.size();
            auto n_z = values_z.size();
            if ((n_x >= 2 && n_y < 2 && n_z < 2) || (n_y >= 2 && n_z < 2 && n_x < 2) || (n_z >= 2 && n_x < 2 && n_y < 2)) {
                throw std::logic_error("Graph::Graph : there must be at least two axes with at least two values and all axes must have at least one value");
            }
            if (values_threat.size() != n_x || values_threat[0].size() != n_y || values_threat[0][0].size() != n_z) {
                throw std::logic_error("Graph::Graph : _values_threat must have the same dimensions as _x, _y, _z");
            }
            if (values_obstacles.size() != n_x || values_obstacles[0].size() != n_y || values_obstacles[0][0].size() != n_z) {
                throw std::logic_error("Graph::Graph : _values_obstacles must have the same dimensions as _x, _y, _z");
            }

            n_i = static_cast<int>(n_x);
            n_j = static_cast<int>(n_y);
            n_z = static_cast<int>(n_z);

            (void) compute_keys_points_vertices();
            (void) compute_edges();

            auto toc = std::clock();
            runtime_build = (toc - tic) / CLOCKS_PER_SEC;
        }


        /**
         * @brief checks if the provided indices are valid for accessing the graph.
         * 
         * @param _i the index along the x-axis.
         * @param _j the index along the y-axis.
         * @param _k the index along the z-axis.
         * @return true if the indices are within the valid range, false otherwise.
         */
        bool are_indices_valid(const int &_i, const int &_j, const int &_k) {
            return (_i >= 0 && _i < n_i && _j >= 0 && _j < n_j && _k >= 0 && _k < n_k);
        }


        /**
         * @brief checks if the provided Key3D pointer is valid for accessing the graph.
         * 
         * @param _key a shared pointer to the Key3D structure.
         * @return true if the key's indices are within the valid range, false otherwise.
         */
        bool are_indices_valid(const std::shared_ptr<Key3D> &_key) {
            return (_key->i >= 0 && _key->i < n_i && _key->j >= 0 && _key->j < n_j && _key->k >= 0 && _key->k < n_k);
        }


        /**
         * @brief finds the linear index of the key based on the given 3D indices.
         * 
         * @param _i the index along the x-axis.
         * @param _j the index along the y-axis.
         * @param _k the index along the z-axis.
         * @return the linear index corresponding to the provided indices.
         */
        int find_index_key(const int &_i, const int &_j, const int &_k) {
            return _i * n_j * n_k + _j * n_k + _k;
        }


        /**
         * @brief finds the linear index of the point based on the given 3D indices.
         * 
         * @param _i the index along the x-axis.
         * @param _j the index along the y-axis.
         * @param _k the index along the z-axis.
         * @return the linear index corresponding to the provided indices.
         */
        int find_index_point(const int &_i, const int &_j, const int &_k) {
            return find_index_key(_i, _j, _k);
        }


        /**
         * @brief looks up vertices at the specified index point.
         * 
         * @param _index_point the linear index of the point.
         * @return a vector of shared pointers to the vertices at the given index point.
         */
        std::vector<std::shared_ptr<Vertex>> lookup_vertices(int &_index_point) {
            auto iterator = vertices_by_index_point.find(_index_point);
            if (iterator != vertices_by_index_point.end()) {
                return iterator->second;
            }
            return std::vector<std::shared_ptr<Vertex>>();
        }


        /**
         * @brief looks up edges at the specified index vertex.
         * 
         * @param _index_vertex the linear index of the vertex.
         * @return a vector of shared pointers to the edges connected to the vertex.
         */
        std::vector<std::shared_ptr<Edge>> lookup_edges(int &_index_vertex) {
            auto iterator = edges_by_index_vertex.find(_index_vertex);
            if (iterator != edges_by_index_vertex.end()) {
                return iterator->second;
            }
            return std::vector<std::shared_ptr<Edge>>();
        }


        /**
         * @brief looks up edges at the specified index vertex, filtering based on the previous edge type.
         * 
         * @param _index_vertex the linear index of the vertex.
         * @param _type_prev the type of the previous edge (to guide filtering).
         * @return a vector of shared pointers to the valid edges that satisfy the condition.
         */
        std::vector<std::shared_ptr<Edge>> lookup_edges(int &_index_vertex, edge_type &_type_prev) {
            std::vector<std::shared_ptr<Edge>> edges_valid;

            auto edges_possible = lookup_edges(_index_vertex);
            for (const auto &edge_possible : edges_possible) {
                // can only translate in plane after rotation (reduces number of expanded nodes)
                if (_type_prev == edge_type::ROTATE && edge_possible->type == edge_type::TRANSLATE_IN_PLANE) {
                    edges_valid.push_back(edge_possible);
                } else {
                    edges_valid.push_back(edge_possible);
                }
            }
            return edges_valid;
        }


        /**
         * @brief looks up the value of the x-coordinate at the ith entry.
         * 
         * @param _i the column index of the x-coordinate.
         * @return the value of the x-coordinate.
         */
        double lookup_x(int _i) {
            if (_i < 0 || _i >= n_i) {
                return -1;
            }
            return *values_x[_i];
        }


        /**
         * @brief looks up the value of the y-coordinate at the jth entry.
         * 
         * @param _j the row index of the y-coordinate.
         * @return the value of the j-coordinate.
         */
        double lookup_y(int _j) {
            if (_j < 0 || _j >= n_j) {
                return -1;
            }
            return *values_y[_j];
        }


        /**
         * @brief looks up the value of the z-coordinate at the kth entry.
         * 
         * @param _k the plane index of the z-coordinate.
         * @return the value of the z-coordinate.
         */
        double lookup_z(int _k) {
            if (_k < 0 || _k >= n_k) {
                return -1;
            }
            return *values_z[_k];
        }


        /**
         * 
         */
        double lookup_threat(int _i, int _j, int _k) {
            if (_i < 0 || _i >= n_i || _j < 0 || _j >= n_j || _k < 0 || _k >= n_k) {
                return -1;
            }
            return *(values_threat[_i][_j][_k]);
        }


        /**
         * 
         */
        std::shared_ptr<Key3D> lookup_key(int _index_key) {
            if (_index_key < 0 || _index_key >= n_keys) {
                return nullptr;
            }
            return keys[_index_key];
        }


        /**
         * 
         */
        std::shared_ptr<Point3D> lookup_point(int _index_point) {
            if (_index_point < 0 || _index_point >= n_points) {
                return nullptr;
            }
            return points[_index_point];
        }


        /**
         * 
         */
        std::shared_ptr<Vertex> lookup_vertex(int _index_vertex) {
            if (_index_vertex < 0 || _index_vertex >= n_vertices) {
                return nullptr;
            }
            return vertices[_index_vertex];
        }


        /**
         * 
         */
        std::shared_ptr<Edge> lookup_edge(int _index_edge) {
            if (_index_edge < 0 || _index_edge >= n_edges) {
                return nullptr;
            }
            return edges[_index_edge];
        }


    private:
        Mesh3D values_threat; // a Mesh3D of static threat values of the graph.
        Mesh3D values_obstacles; // a Mesh3D of static obstacles values of the graph.
        Linspace1D values_x; // a Linspace1D of x values of the graph.
        Linspace1D values_y; // a Linspace1D of y values of the graph.
        Linspace1D values_z; // a Linspace1D of z values of the graph.

        std::vector<std::shared_ptr<Key3D>> keys; // a vector of shared Key3D pointers.
        std::vector<std::shared_ptr<Point3D>> points; // a vector of shared Point3D pointers.
        std::vector<std::shared_ptr<Vertex>> vertices; // a vector of shared Vertex pointers.
        std::vector<std::shared_ptr<Edge>> edges; // a vector of shared Edge pointers.

        boost::unordered_map<int, std::vector<std::shared_ptr<Vertex>>> vertices_by_index_point;
        boost::unordered_map<int, std::vector<std::shared_ptr<Edge>>> edges_by_index_vertex;


        /**
         * @brief computes shared Key3D, Point3D, Vertex pointers for the current graph and fills keys, points, vertices.
         */
        void compute_keys_points_vertices();


        /**
         * @brief computes shared Edge pointers for the current graph and fills edges.
         */
        void compute_edges();
    };


    /**
     * @brief generates a linearly spaced vector.
     * 
     * @param _n_min the minimum value of the linearly spaced vector.
     * @param _n_max the maximum value of the linearly spaced vector.
     * @param _n_n the number of elements in the linearly spaced vector.
     * @return a Linspace1D instance.
     */
    Linspace1D generate_linspace1d(const double &_n_min, const double &_n_max, const int &_n_n);


    /**
     * @brief generates a three-dimensional mesh of uniform values.
     * 
     * @param _n_i the number of columns in the mesh.
     * @param _n_j the number of rows in the mesh.
     * @param _n_k the number of planes in the mesh.
     * @param _value the uniform value to fill the mesh with.
     * @return a Mesh3D instance.
     */
    Mesh3D generate_mesh3d_uniform(const int &_n_i, const int &_n_j, const int &_n_k, const double &_value);


} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_GRAPH_HPP
