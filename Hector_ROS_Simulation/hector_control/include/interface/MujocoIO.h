#pragma once

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/time.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_broadcaster.h>

#include <boost/array.hpp>
#include <memory>
#include <string>

#include "IOInterface.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "unitree_legged_msgs/HighState.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include "simulation.h"
#include "interface/KeyBoard.h"

class MujocoIO : public IOInterface {
 public:
  MujocoIO(std::string robot_name,std::shared_ptr<Simulation> sim):IOInterface(),sim_(sim){
    cmdPanel = new KeyBoard();
  }
  ~MujocoIO()=default;
  void sendRecv(const std::shared_ptr<LowlevelCmd> cmd, LowlevelState* state);
  std::shared_ptr<Simulation> sim_;
};
