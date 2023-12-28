#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <string>

#include "../include/common/ControlFSMData.h"
#include "../include/common/OrientationEstimator.h"
#include "../include/common/PositionVelocityEstimator.h"
#include "../include/interface/CheatIO.h"
#include "../include/FSM/FSM.h"

#include "std_msgs/Bool.h"

bool running = true;
bool pauseFlag = true;

void ShutDown(int sig) {
  std::cout << "stop" << std::endl;
  running = false;
}

void callback(std_msgs::Bool::ConstPtr msg) {
  if (msg->data) {
    std::cout << "pause" << std::endl;
    pauseFlag = true;
  } else {
    std::cout << "resume" << std::endl;
    pauseFlag = false;
  }
}

int main(int argc, char** argv) {
  IOInterface* ioInter;
  ros::init(argc, argv, "pai_control", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  ros::Subscriber simStatesub = nh.subscribe("/pauseFlag", 1, callback);
  ros::AsyncSpinner subSpinner(4);  // one threads
  subSpinner.start();
  std::string robot_name = "pai";
  std::cout << "robot name " << robot_name << std::endl;

  ioInter = new CheatIO(robot_name,nh);
  ros::Rate rate(1000);

  double dt = 0.001;
  Biped biped;

  LegController* legController = new LegController(biped);
  std::shared_ptr<LowlevelCmd> cmd = std::make_shared<LowlevelCmd>();
  LowlevelState* state = new LowlevelState();

  std::cout << "start setup " << std::endl;
  StateEstimate stateEstimate;
  StateEstimatorContainer* stateEstimator = new StateEstimatorContainer(state, legController->data, &stateEstimate);

  stateEstimator->addEstimator<CheaterOrientationEstimator>(); //用于估计机器人的姿态（包括四元数和欧拉角）和角速度（包括世界坐标系下的角速度和机体坐标系下的角速度）
  stateEstimator->addEstimator<CheaterPositionVelocityEstimator>(); //用于估计机器人的位置和速度（包括世界坐标系下的速度和机体坐标系下的速度）

  std::cout << "setup state etimator" << std::endl;

  DesiredStateCommand* desiredStateCommand = new DesiredStateCommand(&stateEstimate, dt);

  std::shared_ptr<ControlFSMData> _controlData = std::make_shared<ControlFSMData>();

  _controlData->_biped = &biped;
  _controlData->_stateEstimator = stateEstimator;
  _controlData->_legController = legController;
  _controlData->_desiredStateCommand = desiredStateCommand;
  _controlData->_interface = ioInter;
  _controlData->_lowCmd = cmd;
  _controlData->_lowState = state;

  std::shared_ptr<FSM> _FSMController = std::make_shared<FSM>(_controlData);

  // signal(SIGINT, ShutDown);

  while (ros::ok()) {
    if (!pauseFlag) _FSMController->run();
    rate.sleep();
  }
  ros::waitForShutdown();

  return 0;

}
