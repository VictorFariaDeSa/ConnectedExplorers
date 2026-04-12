cmake_minimum_required(VERSION 3.8)
project(connected_explorers_global_supervisor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(connected_explorers_utils REQUIRED)
find_package(connected_explorers_interfaces REQUIRED)
find_package(connected_explorers_laplacian_matrix REQUIRED)

add_executable(supervisor_node 
  src/GlobalSupervisorNode.cpp
)

target_include_directories(supervisor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(supervisor_node 
  rclcpp 
  std_msgs 
  geometry_msgs
  connected_explorers_utils
  connected_explorers_interfaces
)

target_link_libraries(supervisor_node 
  connected_explorers_utils::multi_robot_utils
  connected_explorers_utils::map_utils
  connected_explorers_utils::conn_weight_utils
  connected_explorers_laplacian_matrix::laplacian_matrix_utils
)

install(TARGETS
  supervisor_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
