
#include "rclcpp/rclcpp.hpp"
#include "mess2_algorithm_plugins/cbs/solver.hpp"

class TestX : public rclcpp::Node
{
public:
    std::string path_edges;
    std::string path_vertices;
    std::string path_actors;
    std::string path_goals;
    bool result = false;

    TestX() : Node("testx")
    {
        this->declare_parameter("x_min", -5.0);
        this->declare_parameter("x_max", 5.0);
        this->declare_parameter("y_min", -5.0);
        this->declare_parameter("y_max", 5.0);
        this->declare_parameter("z_min", 0.0);
        this->declare_parameter("z_max", 0.0);
        this->declare_parameter("n_x", 201);
        this->declare_parameter("n_y", 201);
        this->declare_parameter("n_z", 1);
        this->declare_parameter("use_diagonals_in_plane", false);

        this->get_parameter("x_min", x_min);
        this->get_parameter("x_max", x_max);
        this->get_parameter("y_min", y_min);
        this->get_parameter("y_max", y_max);
        this->get_parameter("z_min", z_min);
        this->get_parameter("z_max", z_max);
        this->get_parameter("n_x", n_x);
        this->get_parameter("n_y", n_y);
        this->get_parameter("n_z", n_z);
        this->get_parameter("use_diagonals_in_plane", use_diagonals_in_plane);

        this->declare_parameter("path_edges", "~/Projets/ROS2/mess2/matlab/algorithms/config/edges.csv");
        this->declare_parameter("path_vertices", "~/Projets/ROS2/mess2/matlab/algorithms/config/vertices.csv");
        this->declare_parameter("path_actors", "~/Projets/ROS2/mess2/matlab/algorithms/config/actors/");
        this->declare_parameter("path_goals", "~/Projets/ROS2/mess2/matlab/algorithms/config/goals/");

        this->get_parameter("path_edges", path_edges);
        this->get_parameter("path_vertices", path_vertices);
        this->get_parameter("path_actors", path_actors);
        this->get_parameter("path_goals", path_goals);
    }

    void run()
    {
        // create vector representations of mesh values
        RCLCPP_INFO(this->get_logger(), "generating meshes");
        auto x_mesh = mess2_algorithms::generate_linspace1d(x_min, x_max, n_x);
        auto y_mesh = mess2_algorithms::generate_linspace1d(y_min, y_max, n_y);
        auto z_mesh = mess2_algorithms::generate_linspace1d(z_min, z_max, n_z);

        // generate static uniform threat where sum of threat at each point equals one and uniform empty obstacle field
        RCLCPP_INFO(this->get_logger(), "generating uniform threat and obstacle fields");
        auto threat_static_uniform = mess2_algorithms::generate_mesh3d_uniform(n_x, n_y, n_z, 1.0);
        auto obstacles_static_uniform = mess2_algorithms::generate_mesh3d_uniform(n_x, n_y, n_z, 0.0);

        // create dummy graph
        RCLCPP_INFO(this->get_logger(), "generating graph");
        auto graph = std::make_shared<mess2_algorithms::Graph>(x_mesh, y_mesh, z_mesh, threat_static_uniform, obstacles_static_uniform, use_diagonals_in_plane);
        RCLCPP_INFO(this->get_logger(), "\truntime build graph : %f", graph->runtime_build);

        // create dummy actor1
        RCLCPP_INFO(this->get_logger(), "generating actor1");
        auto actor1 = std::make_shared<mess2_algorithms::Actor>(
            graph,
            graph->find_index_vertex_by_xyz_and_heading(-5.0, 0.0, 0.0, 0.0), 
            graph->find_index_vertex_by_xyz_and_heading(0.0, 4.0, 0.0, 0.0),
            0.1, 1.0, 1.0, 0.01, 0.01, 1.0, 1.0
        );
        actor1->name = "actor1";
        RCLCPP_INFO(this->get_logger(), "\truntime build actor : %f", actor1->runtime_build);
        RCLCPP_INFO(this->get_logger(), "\truntime build ot : %f", actor1->runtime_build_ot);

        // create dummy actor2
        RCLCPP_INFO(this->get_logger(), "generating actor2");
        auto actor2 = std::make_shared<mess2_algorithms::Actor>(
            graph,
            graph->find_index_vertex_by_xyz_and_heading(-5.0, -5.0, 0.0, 90.0), 
            graph->find_index_vertex_by_xyz_and_heading(-5.0, 5.0, 0.0, 90.0),
            0.1, 1.0, 1.0, 0.01, 0.01, 1.0, 1.0
        );
        actor2->name = "actor2";
        RCLCPP_INFO(this->get_logger(), "\truntime build actor : %f", actor2->runtime_build);
        RCLCPP_INFO(this->get_logger(), "\truntime build ot : %f", actor2->runtime_build_ot);

        // create dummy instance
        RCLCPP_INFO(this->get_logger(), "generating instance");
        auto instance = std::make_shared<mess2_algorithms::Instance>(graph);
        (void) instance->add_actor(actor1);
        (void) instance->add_actor(actor2);
        
        
        

        // create dummy cbs solver
        RCLCPP_INFO(this->get_logger(), "generating cbs solver");
        auto cbs = mess2_algorithms::CBS(instance, false, 0);
        RCLCPP_INFO(this->get_logger(), "\truntime build preprocess : %f", cbs.runtime_preprocessing);

        result = cbs.solve(10.0);










        if (result) {
            RCLCPP_INFO(this->get_logger(), "saving actors");
            (void) actor1->save_actor(path_actors + "actor1.csv");
            (void) actor2->save_actor(path_actors + "actor2.csv");

            RCLCPP_INFO(this->get_logger(), "saving paths");
            (void) cbs.save_paths(path_goals, true);
        } else {
            RCLCPP_INFO(this->get_logger(), "algorithm failed");
        }

        RCLCPP_INFO(this->get_logger(), "done");
    }

private:
    double x_min, x_max, y_min, y_max, z_min, z_max;
    int n_x, n_y, n_z;
    bool use_diagonals_in_plane;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestX>();
    (void) node->run();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
