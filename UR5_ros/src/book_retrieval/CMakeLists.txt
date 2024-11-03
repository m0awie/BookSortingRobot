cmake_minimum_required(VERSION 3.8)
project(book_retrieval)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BookInfo.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

install(DIRECTORY
  book_retrieval
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  book_retrieval/book_detection_node.py
  book_retrieval/book_move_node.py
  book_retrieval/gripper_control_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclpy sensor_msgs geometry_msgs std_msgs cv_bridge OpenCV rosidl_default_runtime)

ament_package()

