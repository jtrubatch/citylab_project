cmake_minimum_required(VERSION 3.5)
project(robot_patrol)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(direction_service_interface REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(robot_patrol src/patrol.cpp)
ament_target_dependencies(robot_patrol rclcpp geometry_msgs sensor_msgs)

add_executable(direction_server src/direction_service.cpp)
ament_target_dependencies(direction_server rclcpp direction_service_interface)

add_executable(test_service src/test_service.cpp)
ament_target_dependencies(test_service rclcpp direction_service_interface sensor_msgs std_srvs)

add_executable(robot_patrol2 src/patrol_v2.cpp)
ament_target_dependencies(robot_patrol2 rclcpp geometry_msgs sensor_msgs direction_service_interface)

install(TARGETS
   robot_patrol
   direction_server
   test_service
   robot_patrol2
   DESTINATION lib/${PROJECT_NAME}
 )

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
