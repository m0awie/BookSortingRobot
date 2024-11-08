cmake_minimum_required(VERSION 3.8)
project(robot_brain)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)

add_executable(brain_node src/brain_node.cpp)
ament_target_dependencies(brain_node rclcpp interfaces)

install(TARGETS
  brain_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
