cmake_minimum_required(VERSION 3.8)
project(factr_controller)

#-------------------------------#
# Compiler / C++ standard
#-------------------------------#
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#-------------------------------#
# Dependencies
#-------------------------------#
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(dynamixel_sdk REQUIRED)  # <= 추가
find_package(pinocchio REQUIRED)

# include directory
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

#-------------------------------#
# Library (Controller Components)
#-------------------------------#
add_library(factr_controller_lib
  src/dynamixel_sdk_interface.cpp
  src/admittance_controller.cpp
  src/external_torque_estimator.cpp
  src/robot_arm.cpp
)

ament_target_dependencies(factr_controller_lib
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  dynamixel_sdk        # <= 추가
  pinocchio
  Eigen3
)

#-------------------------------#
# Executable Node
#-------------------------------#
add_executable(robot_arm_node src/robot_arm_node.cpp)

target_link_libraries(robot_arm_node
  factr_controller_lib
)

ament_target_dependencies(robot_arm_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
)

#-------------------------------#
# Install
#-------------------------------#
install(TARGETS
  factr_controller_lib
  robot_arm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

#-------------------------------#
# ament package
#-------------------------------#
ament_package()
