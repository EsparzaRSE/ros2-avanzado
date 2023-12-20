cmake_minimum_required(VERSION 3.8)
project(actions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(my_robot_interfaces REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# crear el ejecutable e instalar el nodo
#add_executable(tortuga_spawner2 src/tortuga_spawner2.cpp)
#ament_target_dependencies(tortuga_spawner2 rclcpp turtlesim my_robot_interfaces)


install(TARGETS 
  #tortuga_spawner2
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
