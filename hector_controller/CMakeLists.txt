cmake_minimum_required(VERSION 3.5)
project(hector_controller)

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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(hector_description REQUIRED)

#find_package(pinocchio REQUIRED)
#find_package(controller_manager REQUIRED)
#find_package(joint_state_controller REQUIRED)
#find_package(robot_state_publisher REQUIRED)
#find_package(mujoco)

add_subdirectory(third_party/qpOASES)

include_directories(${CMAKE_BINARY_DIR})
include_directories(
    include
)

#file(GLOB_RECURSE SRC_LIST
#    "src/*/*.cpp"
#    "src/*/*.h"
#    "convex_mpc/*.cpp"
#)
#list(REMOVE_ITEM SRC_LIST "src/interface/MujocoIO.cpp")
#
#add_executable(hector_ctrl src/main.cpp ${SRC_LIST} )
#target_link_libraries(hector_ctrl rclcpp qpOASES -pthread)
#
#install(TARGETS
#   hector_ctrl
#   DESTINATION lib/${PROJECT_NAME}
#)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
