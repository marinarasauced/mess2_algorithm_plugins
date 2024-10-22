cmake_minimum_required(VERSION 3.8)
project(mess2_algorithm_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

include_directories(include)

add_library(_mess2_algorithm_plugins src/graph.cpp)

# add_library(_mess2_algorithm_plugins_actor src/actor.cpp)
# add_library(_mess2_algorithm_plugins_constraint src/constraint.cpp)
# add_library(_mess2_algorithm_plugins_graph src/graph.cpp)
# add_library(_mess2_algorithm_plugins_hl_queue src/high_level_queue.cpp)
# add_library(_mess2_algorithm_plugins_hl_search src/high_level_search.cpp)
# add_library(_mess2_algorithm_plugins_ll_history src/low_level_history.cpp)
# add_library(_mess2_algorithm_plugins_ll_queue src/low_level_queue.cpp)
# add_library(_mess2_algorithm_plugins_ll_search src/low_level_search.cpp)

# target_link_libraries(
#   _mess2_algorithm_plugins_actor
#   _mess2_algorithm_plugins_graph
# )

# target_link_libraries(
#   _mess2_algorithm_plugins_ll_search
#   _mess2_algorithm_plugins_actor
#   _mess2_algorithm_plugins_constraint
#   _mess2_algorithm_plugins_graph
#   _mess2_algorithm_plugins_ll_history
#   _mess2_algorithm_plugins_ll_queue
# )

# target_link_libraries(
#   _mess2_algorithm_plugins_hl_queue
#   _mess2_algorithm_plugins_constraint
# )

# target_link_libraries(
#   _mess2_algorithm_plugins_hl_search
#   _mess2_algorithm_plugins_actor
#   _mess2_algorithm_plugins_constraint
#   _mess2_algorithm_plugins_graph
#   _mess2_algorithm_plugins_hl_queue
#   _mess2_algorithm_plugins_ll_history
#   _mess2_algorithm_plugins_ll_queue
#   _mess2_algorithm_plugins_ll_search
# )

install(
  TARGETS _mess2_algorithm_plugins
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
