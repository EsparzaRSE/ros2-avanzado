cmake_minimum_required(VERSION 3.8)
project(actions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(my_robot_advanced_interfaces REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# crear el ejecutable e instalar el nodo
add_executable(count_until_server src/count_until_server.cpp)
ament_target_dependencies(count_until_server rclcpp my_robot_advanced_interfaces rclcpp_action)

add_executable(count_until_client src/count_until_client.cpp)
ament_target_dependencies(count_until_client rclcpp my_robot_advanced_interfaces rclcpp_action)

add_executable(count_until_server_queue_goals src/count_until_server_queue_goals.cpp)
ament_target_dependencies(count_until_server_queue_goals rclcpp my_robot_advanced_interfaces rclcpp_action)

install(TARGETS 
  count_until_server
  count_until_client
  count_until_server_queue_goals
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
