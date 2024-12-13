cmake_minimum_required(VERSION 3.8)
project(mess2_algorithm_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mess2_algorithm_threat_static REQUIRED)

include_directories(
  include
  ${mess2_algorithm_threat_static_INCLUDE_DIRS}
)

add_library(
  _mess2_algorithm_plugins
  src/common.cpp
  src/graph.cpp
  src/actor.cpp
  src/instance.cpp
  src/conflict.cpp
  src/cbs/node.cpp
  src/constraint_table.cpp
  src/astar/single_actor_solver.cpp
  src/astar/space_time_a_star.cpp
  src/cbs/solver.cpp
)

add_executable(demo1 src/demos/demo1.cpp)
add_executable(demo2 src/demos/demo2.cpp)
add_executable(demo3 src/demos/demo3.cpp)

target_link_libraries(
  demo1
  _mess2_algorithm_plugins
)

target_link_libraries(
  demo2
  _mess2_algorithm_plugins
)

target_link_libraries(
  demo3
  _mess2_algorithm_plugins
)

ament_target_dependencies(
  demo1
  "rclcpp"
  "mess2_algorithm_threat_static"
)

ament_target_dependencies(
  demo2
  "rclcpp"
  "mess2_algorithm_threat_static"
)

ament_target_dependencies(
  demo3
  "rclcpp"
  "mess2_algorithm_threat_static"
)

install(
  TARGETS _mess2_algorithm_plugins
  EXPORT export_mess2_algorithm_plugins
  LIBRARY DESTINATION lib
)

install(
  TARGETS demo1 demo2 demo3
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_mess2_algorithm_plugins)
ament_package()
