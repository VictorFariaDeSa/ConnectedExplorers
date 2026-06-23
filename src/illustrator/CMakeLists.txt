cmake_minimum_required(VERSION 3.8)
project(illustrator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(connected_explorers_interfaces REQUIRED)
find_package(connected_explorers_utils REQUIRED)

add_executable(illustrator_node 
  src/IllustratorNode.cpp
  src/MarkerDesigner.cpp
  src/ColorClassifier.cpp
)

target_include_directories(illustrator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(illustrator_node 
  rclcpp 
  std_msgs 
  visualization_msgs 
  connected_explorers_interfaces
  connected_explorers_utils
)

target_link_libraries(illustrator_node 
  connected_explorers_utils::multi_robot_utils
)

install(TARGETS
  illustrator_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
