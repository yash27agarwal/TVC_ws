cmake_minimum_required(VERSION 3.8)
project(px4_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs)

# Add the frame transforms library
add_library(frame_transforms SHARED
  src/lib/frame_transforms.cpp
)

# Set include directories for the library
target_include_directories(frame_transforms PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# Set dependencies for the library
ament_target_dependencies(frame_transforms
  Eigen3
  geometry_msgs
  sensor_msgs
  std_msgs
  rclcpp
)

# Add the main executable (px4_test)
add_executable(px4_test src/px4_test_node.cpp)

# Set include directories for the px4_test executable
target_include_directories(px4_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lib
)

# Set dependencies for the px4_test executable
ament_target_dependencies(px4_test 
  rclcpp 
  px4_msgs
  Eigen3
  geometry_msgs
  sensor_msgs
)

# Link the frame_transforms library to the px4_test executable
target_link_libraries(px4_test
  frame_transforms
)

# Add the gazebo_clock executable
add_executable(gazebo_clock src/gazebo_clock.cpp)

# Set include directories for the gazebo_clock executable
target_include_directories(gazebo_clock PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lib
)

# Set dependencies for the gazebo_clock executable
ament_target_dependencies(gazebo_clock 
  rclcpp 
  px4_msgs
  Eigen3
  geometry_msgs
  std_msgs
  sensor_msgs
)

# Link the frame_transforms library to the gazebo_clock executable (if needed)
target_link_libraries(gazebo_clock
  frame_transforms
)

# Install includes
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install the config directory
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install targets
install(TARGETS
  frame_transforms
  px4_test
  gazebo_clock
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3
  geometry_msgs
  sensor_msgs
  rclcpp
  px4_msgs
  std_msgs
)

ament_package()