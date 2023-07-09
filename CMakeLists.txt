cmake_minimum_required(VERSION 3.8)
project(ros2-gst-test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(server_node src/server.cpp)
ament_target_dependencies(server_node rclcpp std_msgs std_srvs)

add_executable(client_node src/client.cpp)
ament_target_dependencies(client_node rclcpp std_msgs std_srvs)

install(TARGETS
  server_node
  client_node
   DESTINATION lib/${PROJECT_NAME}
 )

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
