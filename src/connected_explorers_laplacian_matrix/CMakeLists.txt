cmake_minimum_required(VERSION 3.8)
project(connected_explorers_laplacian_matrix)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)


# --- Laplacian Matrix Utils ---
add_library(laplacian_matrix_utils src/LaplacianMatrixHandler.cpp)
target_include_directories(laplacian_matrix_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Use Eigen directly for the library
target_link_libraries(laplacian_matrix_utils Eigen3::Eigen)

# CRITICAL: The library needs to know about ROS 2 headers too!
ament_target_dependencies(laplacian_matrix_utils 
  rclcpp 
  std_msgs
)

# --- The Node (The Executable) ---
add_executable(laplacian_estimator_node 
  src/LaplacianMatrixEstimatorNode.cpp
)

# Link your internal library and external dependencies
target_link_libraries(laplacian_estimator_node laplacian_matrix_utils)

ament_target_dependencies(laplacian_estimator_node 
  rclcpp 
  std_msgs 
  Eigen3
)

ament_export_targets(export_laplacian_matrix_utils HAS_LIBRARY_TARGET)

ament_export_dependencies(Eigen3 OpenCV rclcpp std_msgs)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS laplacian_matrix_utils
  EXPORT export_laplacian_matrix_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS laplacian_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
