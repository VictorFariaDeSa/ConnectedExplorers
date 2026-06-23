cmake_minimum_required(VERSION 3.8)
project(connected_explorers_connections)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(connected_explorers_interfaces REQUIRED)
find_package(connected_explorers_utils REQUIRED)
find_package(PCL REQUIRED)


add_executable(distance_watcher_node 
  src/DistanceWatcherNode.cpp
)


target_include_directories(distance_watcher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)


ament_target_dependencies(distance_watcher_node 
  rclcpp 
  geometry_msgs 
  sensor_msgs 
  connected_explorers_interfaces
  connected_explorers_utils
)

target_link_libraries(distance_watcher_node 
  connected_explorers_utils::multi_robot_utils
  ${PCL_LIBRARIES}
)

target_compile_definitions(distance_watcher_node PUBLIC ${PCL_DEFINITIONS})

install(TARGETS
  distance_watcher_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
