
#include "rclcpp/rclcpp.hpp"
#include "mess2_algorithm_plugins/cbs/solver.hpp"
#include "mess2_algorithm_threat_static/threat.hpp"

class TestX : public rclcpp::Node
{
public:
    std::string path_edges;
    std::string path_vertices;
    std::string path_actors;
    std::string path_goals;
    bool result = false;

    TestX() : Node("test3")
    {
        this->declare_parameter("x_min", -1.5);
        this->declare_parameter("x_max", 1.5);
        this->declare_parameter("y_min", -1.5);
        this->declare_parameter("y_max", 1.5);
        this->declare_parameter("z_min", 0.0);
        this->declare_parameter("z_max", 0.0);
        this->declare_parameter("n_x", 31);
        this->declare_parameter("n_y", 31);
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
        auto threat_static = mess2_algorithms::generate_threat_static_2d(n_x, n_y, n_z);
        auto obstacles_static_uniform = mess2_algorithms::generate_mesh3d_uniform(n_x, n_y, n_z, 0.0);

        // create dummy graph
        RCLCPP_INFO(this->get_logger(), "generating graph");
        auto graph = std::make_shared<mess2_algorithms::Graph>(x_mesh, y_mesh, z_mesh, threat_static, obstacles_static_uniform, use_diagonals_in_plane);
        RCLCPP_INFO(this->get_logger(), "\truntime build graph : %f", graph->runtime_build);

        // create burger2
        RCLCPP_INFO(this->get_logger(), "generating burger2");
        auto burger2 = std::make_shared<mess2_algorithms::Actor>(
            graph,
            graph->find_index_vertex_by_xyz_and_heading(1.4, -1.4, 0.0, 0.0), 
            graph->find_index_vertex_by_xyz_and_heading(1.0, -1.4, 0.0, 0.0),
            0.15, 2.0, 5.0, 0.01, 0.02, 2.84, 0.22
        );
        burger2->name = "burger2";
        RCLCPP_INFO(this->get_logger(), "\truntime build actor : %f", burger2->runtime_build);
        RCLCPP_INFO(this->get_logger(), "\truntime build ot : %f", burger2->runtime_build_ot);

        // create burger3
        RCLCPP_INFO(this->get_logger(), "generating burger3");
        auto burger3 = std::make_shared<mess2_algorithms::Actor>(
            graph,
            graph->find_index_vertex_by_xyz_and_heading(1.1, -1.4, 0.0, 0.0), 
            graph->find_index_vertex_by_xyz_and_heading(1.5, -1.4, 0.0, 0.0),
            0.15, 2.0, 5.0, 0.01, 0.02, 2.84, 0.22
        );
        burger3->name = "burger3";
        RCLCPP_INFO(this->get_logger(), "\truntime build actor : %f", burger3->runtime_build);
        RCLCPP_INFO(this->get_logger(), "\truntime build ot : %f", burger3->runtime_build_ot);

        // create wafflepi2
        RCLCPP_INFO(this->get_logger(), "generating wafflepi2");
        auto wafflepi2 = std::make_shared<mess2_algorithms::Actor>(
            graph,
            graph->find_index_vertex_by_xyz_and_heading(0.0, 0.0, 0.0, 0.0), 
            graph->find_index_vertex_by_xyz_and_heading(-0.7, 1.4, 0.0, 0.0),
            0.30, 2.0, 5.0, 0.01, 0.02, 1.82, 0.26
        );
        wafflepi2->name = "wafflepi2";
        RCLCPP_INFO(this->get_logger(), "\truntime build actor : %f", wafflepi2->runtime_build);
        RCLCPP_INFO(this->get_logger(), "\truntime build ot : %f", wafflepi2->runtime_build_ot);

        // create wafflepi1
        RCLCPP_INFO(this->get_logger(), "generating wafflepi1");
        auto wafflepi1 = std::make_shared<mess2_algorithms::Actor>(
            graph,
            graph->find_index_vertex_by_xyz_and_heading(-1.0, 0.5, 0.0, 0.0), 
            graph->find_index_vertex_by_xyz_and_heading(0.0, 0.0, 0.0, 0.0),
            0.30, 2.0, 5.0, 0.01, 0.02, 1.82, 0.26
        );
        wafflepi1->name = "wafflepi1";
        RCLCPP_INFO(this->get_logger(), "\truntime build actor : %f", wafflepi1->runtime_build);
        RCLCPP_INFO(this->get_logger(), "\truntime build ot : %f", wafflepi1->runtime_build_ot);

        // create wafflepi3
        RCLCPP_INFO(this->get_logger(), "generating wafflepi3");
        auto wafflepi3 = std::make_shared<mess2_algorithms::Actor>(
            graph,
            graph->find_index_vertex_by_xyz_and_heading(1.1, 1.1, 0.0, 0.0), 
            graph->find_index_vertex_by_xyz_and_heading(1.2, -0.9, 0.0, 0.0),
            0.30, 2.0, 5.0, 0.01, 0.02, 1.82, 0.26
        );
        wafflepi3->name = "wafflepi3";
        RCLCPP_INFO(this->get_logger(), "\truntime build actor : %f", wafflepi3->runtime_build);
        RCLCPP_INFO(this->get_logger(), "\truntime build ot : %f", wafflepi3->runtime_build_ot);

        // create wafflepi4
        RCLCPP_INFO(this->get_logger(), "generating wafflepi4");
        auto wafflepi4 = std::make_shared<mess2_algorithms::Actor>(
            graph,
            graph->find_index_vertex_by_xyz_and_heading(-0.6, -1.2, 0.0, 0.0), 
            graph->find_index_vertex_by_xyz_and_heading(-1.2, 0.5, 0.0, 0.0),
            0.30, 2.0, 5.0, 0.01, 0.02, 1.82, 0.26
        );
        wafflepi4->name = "wafflepi4";
        RCLCPP_INFO(this->get_logger(), "\truntime build actor : %f", wafflepi4->runtime_build);
        RCLCPP_INFO(this->get_logger(), "\truntime build ot : %f", wafflepi4->runtime_build_ot);

        // create burger4
        RCLCPP_INFO(this->get_logger(), "generating burger4");
        auto burger4 = std::make_shared<mess2_algorithms::Actor>(
            graph,
            graph->find_index_vertex_by_xyz_and_heading(-0.8, 1.2, 0.0, 0.0), 
            graph->find_index_vertex_by_xyz_and_heading(-1.3, -1.3, 0.0, 0.0),
            0.15, 2.0, 5.0, 0.01, 0.02, 2.84, 0.22
        );
        burger4->name = "burger4";
        RCLCPP_INFO(this->get_logger(), "\truntime build actor : %f", burger4->runtime_build);
        RCLCPP_INFO(this->get_logger(), "\truntime build ot : %f", burger4->runtime_build_ot);

        // // create dummy actor1
        // RCLCPP_INFO(this->get_logger(), "generating actor1");
        // auto actor1 = std::make_shared<mess2_algorithms::Actor>(
        //     graph,
        //     graph->find_index_vertex_by_xyz_and_heading(-5.0, 0.0, 0.0, 0.0), 
        //     graph->find_index_vertex_by_xyz_and_heading(0.0, 4.0, 0.0, 0.0),
        //     2.0, 1.0, 1.0, 0.01, 0.01, 1.0, 1.0
        // );
        // actor1->name = "actor1";
        // RCLCPP_INFO(this->get_logger(), "\truntime build actor : %f", actor1->runtime_build);
        // RCLCPP_INFO(this->get_logger(), "\truntime build ot : %f", actor1->runtime_build_ot);

        // // create dummy actor2
        // RCLCPP_INFO(this->get_logger(), "generating actor2");
        // auto actor2 = std::make_shared<mess2_algorithms::Actor>(
        //     graph,
        //     graph->find_index_vertex_by_xyz_and_heading(-5.0, -5.0, 0.0, 90.0), 
        //     graph->find_index_vertex_by_xyz_and_heading(-5.0, 5.0, 0.0, 90.0),
        //     1.0, 1.0, 1.0, 0.01, 0.01, 1.0, 1.0
        // );
        // actor2->name = "actor2";
        // RCLCPP_INFO(this->get_logger(), "\truntime build actor : %f", actor2->runtime_build);
        // RCLCPP_INFO(this->get_logger(), "\truntime build ot : %f", actor2->runtime_build_ot);

        // // create dummy actor3
        // RCLCPP_INFO(this->get_logger(), "generating actor3");
        // auto actor3 = std::make_shared<mess2_algorithms::Actor>(
        //     graph,
        //     graph->find_index_vertex_by_xyz_and_heading(-5.0, 5.0, 0.0, 0.0), 
        //     graph->find_index_vertex_by_xyz_and_heading(-3.0, -3.0, 0.0, 270.0),
        //     1.0, 1.0, 1.0, 0.01, 0.01, 1.0, 1.0
        // );
        // actor3->name = "actor3";
        // RCLCPP_INFO(this->get_logger(), "\truntime build actor : %f", actor3->runtime_build);
        // RCLCPP_INFO(this->get_logger(), "\truntime build ot : %f", actor3->runtime_build_ot);

        // // create dummy actor4
        // RCLCPP_INFO(this->get_logger(), "generating actor4");
        // auto actor4 = std::make_shared<mess2_algorithms::Actor>(
        //     graph,
        //     graph->find_index_vertex_by_xyz_and_heading(5.0, -5.0, 0.0, 90.0), 
        //     graph->find_index_vertex_by_xyz_and_heading(-5.0, -5.0, 0.0, 90.0),
        //     0.5, 1.0, 1.0, 0.01, 0.01, 1.0, 2.0
        // );
        // actor4->name = "actor4";
        // RCLCPP_INFO(this->get_logger(), "\truntime build actor : %f", actor4->runtime_build);
        // RCLCPP_INFO(this->get_logger(), "\truntime build ot : %f", actor4->runtime_build_ot);

        // create dummy instance
        RCLCPP_INFO(this->get_logger(), "generating instance");
        auto instance = std::make_shared<mess2_algorithms::Instance>(graph);
        (void) instance->add_actor(burger2);
        (void) instance->add_actor(burger3);
        (void) instance->add_actor(wafflepi2);
        (void) instance->add_actor(wafflepi1);
        (void) instance->add_actor(wafflepi3);
        (void) instance->add_actor(wafflepi4);
        (void) instance->add_actor(burger4);
        // 
        
        
        

        // create dummy cbs solver
        RCLCPP_INFO(this->get_logger(), "generating cbs solver");
        auto cbs = mess2_algorithms::CBS(instance, false, 0);
        RCLCPP_INFO(this->get_logger(), "\truntime build preprocess : %f", cbs.runtime_preprocessing);

        result = cbs.solve(10.0);










        if (result) {
            RCLCPP_INFO(this->get_logger(), "saving actors");
            (void) burger2->save_actor(path_actors + "burger2.csv");
            (void) burger3->save_actor(path_actors + "burger3.csv");
            (void) wafflepi2->save_actor(path_actors + "wafflepi2.csv");
            (void) wafflepi1->save_actor(path_actors + "wafflepi1.csv");
            (void) wafflepi3->save_actor(path_actors + "wafflepi3.csv");
            (void) wafflepi4->save_actor(path_actors + "wafflepi4.csv");
            (void) burger4->save_actor(path_actors + "burger4.csv");

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
