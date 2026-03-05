cmake_minimum_required(VERSION 3.8)
project(vla_rules_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(fake_vla_stub src/fake_vla_stub.cpp)
ament_target_dependencies(fake_vla_stub rclcpp std_msgs geometry_msgs)

add_executable(arm_executor src/arm_executor.cpp)
ament_target_dependencies(arm_executor rclcpp geometry_msgs moveit_ros_planning_interface)

install(TARGETS
  fake_vla_stub
  arm_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  worlds
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()