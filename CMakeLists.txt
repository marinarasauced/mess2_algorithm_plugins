cmake_minimum_required(VERSION 3.8)
project(mess2_algorithm_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

include_directories(include)

add_library(edge SHARED src/graph/edge.cpp)
add_library(vertex SHARED src/graph/vertex.cpp)
add_library(_graph SHARED src/graph/_graph.cpp)

add_library(adjacency SHARED src/low_level/adjacency.cpp)
add_library(llhistory SHARED src/low_level/history.cpp)
add_library(llqueue SHARED src/low_level/queue.cpp)
add_library(_low_level SHARED src/low_level/_low_level.cpp)

add_library(actor SHARED src/actor.cpp)
add_library(utils SHARED src/utils.cpp)

install(
  TARGETS edge vertex _graph adjacency llhistory llqueue _low_level actor utils
  EXPORT export_mess2_algorithm_plugins
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_mess2_algorithm_plugins)
ament_package()
