#ifndef CHEATIO_H
#define CHEATIO_H

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

class CheatIO : public IOInterface {
 public:
  CheatIO(std::string robot_name,ros::NodeHandle n);
  ~CheatIO();
  void sendRecv(const std::shared_ptr<LowlevelCmd> cmd, LowlevelState* state);

 private:
  void sendCmd(const std::shared_ptr<LowlevelCmd> cmd);
  void recvState(LowlevelState* state);
  ros::NodeHandle& _nm;
  ros::Subscriber _servo_sub[10], _state_sub;
  ros::Publisher _servo_pub[10];
  // * For mujoco simulation
  ros::Subscriber joinsPosVelSub_, bodyPoseSub_, bodyTwistSub_;
  ros::Publisher jointsTorquePub_;
  void jointsPosVelCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
  void bodyPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void bodyTwistCallback(const geometry_msgs::TwistConstPtr& msg);
  void sendCmdMj(const std::shared_ptr<LowlevelCmd> cmd);

  unitree_legged_msgs::LowCmd _lowCmd;
  unitree_legged_msgs::HighState _highState;

  std::string _robot_name;
  void initRecv();  // initialize subscribers
  void initSend();  // initialize publishers

  void StateCallback(const gazebo_msgs::ModelStates& msg);

  void LhipCallback(const unitree_legged_msgs::MotorState& msg);
  void Lhip2Callback(const unitree_legged_msgs::MotorState& msg);
  void LthighCallback(const unitree_legged_msgs::MotorState& msg);
  void LcalfCallback(const unitree_legged_msgs::MotorState& msg);
  void LtoeCallback(const unitree_legged_msgs::MotorState& msg);
  void RhipCallback(const unitree_legged_msgs::MotorState& msg);
  void Rhip2Callback(const unitree_legged_msgs::MotorState& msg);
  void RthighCallback(const unitree_legged_msgs::MotorState& msg);
  void RcalfCallback(const unitree_legged_msgs::MotorState& msg);
  void RtoeCallback(const unitree_legged_msgs::MotorState& msg);
};

#endif