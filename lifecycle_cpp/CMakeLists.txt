cmake_minimum_required(VERSION 3.8)
project(lifecycle_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_advanced_interfaces REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(number_publisher src/number_publisher.cpp)
ament_target_dependencies(number_publisher rclcpp rclcpp_lifecycle example_interfaces)

add_executable(lifecycle_challenge_server src/lifecycle_challenge_server.cpp)
ament_target_dependencies(lifecycle_challenge_server rclcpp rclcpp_lifecycle rclcpp_action my_robot_advanced_interfaces)

install(TARGETS
  number_publisher
  lifecycle_challenge_server
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
