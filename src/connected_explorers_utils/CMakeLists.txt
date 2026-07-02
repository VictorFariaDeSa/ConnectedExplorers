cmake_minimum_required(VERSION 3.8)
project(connected_explorers_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

# --- ADDED: Find Boost explicitly because PCL targets rely on it ---
find_package(Boost REQUIRED COMPONENTS system)

# --- Multi Robot Utils ---
add_library(multi_robot_utils src/MultiRobotsPoseHandler.cpp)
target_include_directories(multi_robot_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(multi_robot_utils rclcpp geometry_msgs)

# --- Map Utils ---
add_library(map_utils src/MapHandler.cpp)
target_include_directories(map_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(map_utils ${OpenCV_LIBRARIES}) 
ament_target_dependencies(map_utils rclcpp geometry_msgs nav_msgs)

# --- Conn Weight Utils ---
add_library(conn_weight_utils src/ConnWeightHandler.cpp)
target_include_directories(conn_weight_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(conn_weight_utils rclcpp geometry_msgs)

# --- Map 3D Utils ---
add_library(map3d_utils src/MapHandler3d.cpp)
target_include_directories(map3d_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(map3d_utils ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
# Adding PCL definitions is recommended as it prevents some obscure build errors
target_compile_definitions(map3d_utils PUBLIC ${PCL_DEFINITIONS}) 
ament_target_dependencies(map3d_utils rclcpp geometry_msgs pcl_conversions)


# --- Export and Install ---
# Add ALL your libraries to the export list and install
ament_export_targets(export_connected_explorers_utils HAS_LIBRARY_TARGET)

# --- MODIFIED: Added Boost, PCL, and pcl_conversions so downstream packages know about them ---
ament_export_dependencies(nav_msgs rclcpp geometry_msgs std_msgs Eigen3 OpenCV PCL pcl_conversions Boost)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS 
  multi_robot_utils 
  map_utils
  conn_weight_utils
  map3d_utils
  EXPORT export_connected_explorers_utils
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()