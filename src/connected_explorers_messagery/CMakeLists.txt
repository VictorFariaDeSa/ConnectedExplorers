cmake_minimum_required(VERSION 3.8)
project(connected_explorers_messagery)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(connected_explorers_interfaces REQUIRED)
find_package(connected_explorers_laplacian_matrix REQUIRED)



add_executable(robot_inbox_node
  src/RobotInboxNode.cpp
)

add_executable(robot_outbox_node
  src/RobotOutboxNode.cpp
)

add_executable(router_node
  src/RouterNode.cpp
)

ament_target_dependencies(robot_inbox_node 
  rclcpp 
  std_msgs
  geometry_msgs 
  connected_explorers_interfaces
  connected_explorers_laplacian_matrix
)

ament_target_dependencies(robot_outbox_node 
  rclcpp 
  std_msgs
  geometry_msgs 
  connected_explorers_interfaces
  connected_explorers_laplacian_matrix
)

ament_target_dependencies(router_node 
  rclcpp 
  std_msgs
  geometry_msgs 
  connected_explorers_interfaces
  connected_explorers_laplacian_matrix
)

install(TARGETS
  robot_inbox_node
  robot_outbox_node
  router_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
