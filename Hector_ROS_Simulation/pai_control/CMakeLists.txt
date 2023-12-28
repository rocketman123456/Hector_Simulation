cmake_minimum_required(VERSION 3.5)
project(pai_control)
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(CMAKE_C_FLAGS "-O3  -ggdb  -march=native -std=gnu99 -I.")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
# set(CMAKE_CXX_FLAGS "-lboost_system -fext-numeric-literals -O3") 
add_definitions(-DEIGEN_STACK_ALLOCATION_LIMIT=0) 
# EIGEN_STACK_ALLOCATION_LIMIT - defines the maximum bytes for a buffer to be allocated on the stack. 
# For internal temporary buffers, dynamic memory allocation is employed as a fall back. For fixed-size
# matrices or arrays, exceeding this threshold raises a compile time assertion. Use 0 to set no limit. 
# Default is 128 KB.
find_package(catkin REQUIRED COMPONENTS
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    roscpp
    rospy
    gazebo_ros
    gazebo_msgs
    std_msgs
    tf
    geometry_msgs
    unitree_legged_msgs  
    hector_description
)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)
find_package(pinocchio REQUIRED)
#find_package(qpoases)
catkin_package(
    CATKIN_DEPENDS 
    unitree_legged_msgs 
)
find_package(mujoco)
find_package(glfw3 REQUIRED)

include_directories(${CMAKE_BINARY_DIR})
include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${pinocchio_INCLUDE_DIRS}
)

link_directories(
    include/sdk/lib
    ${GAZEBO_LIBRARY_DIRS}
)

add_subdirectory(third_party/qpOASES)

file(GLOB_RECURSE SRC_LIST
"src/*/*.cpp"
"src/*/*.h"
"ConvexMPC/*.cpp"
)
list(REMOVE_ITEM SRC_LIST "src/interface/MujocoIO.cpp")
#print all source files
# foreach(_source IN ITEMS ${SRC_LIST})
  #  message(STATUS "source file: ${_source}")
# endforeach()

add_executable(pai_ctrl src/main.cpp  src/simulation.cpp ${SRC_LIST} )
target_link_libraries(pai_ctrl 
  ${catkin_LIBRARIES} qpOASES -pthread lcm 
  mujoco::mujoco 
  glfw
  ${pinocchio_LIBRARIES}
)

add_executable(pino_test src/pinocchio_test.cpp)
target_link_libraries(pino_test ${catkin_LIBRARIES} ${pinocchio_LIBRARIES})

# add_executable(mj_sim_node src/mjSim.cpp src/simulation.cpp 
# src/interface/MujocoIO.cpp
# ${SRC_LIST} )
# target_link_libraries(mj_sim_node ${catkin_LIBRARIES} qpOASES -pthread lcm mujoco::mujoco glfw
# )

