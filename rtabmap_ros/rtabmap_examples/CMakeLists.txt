cmake_minimum_required(VERSION 3.5)
project(rtabmap_examples)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Build C++ IMU axis transformer node
add_executable(imu_axis_transformer_node src/imu_axis_transformer_node.cpp)
ament_target_dependencies(imu_axis_transformer_node
  rclcpp
  sensor_msgs
)

# Install C++ executable
install(TARGETS
  imu_axis_transformer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config scripts
   DESTINATION share/${PROJECT_NAME}
)

file(GLOB EXAMPLE_SCRIPTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*")

install(PROGRAMS 
  ${EXAMPLE_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
