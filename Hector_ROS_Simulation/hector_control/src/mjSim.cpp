#include <GLFW/glfw3.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include <ros/package.h>
#include <sched.h>
#include <std_msgs/Bool.h>
#include <unistd.h>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
#include "../include/FSM/FSM.h"
#include "../include/common/ControlFSMData.h"
#include "../include/common/OrientationEstimator.h"
#include "../include/common/PositionVelocityEstimator.h"
#include "../include/interface/MujocoIO.h"
#include "glfw/glfw_adapter.h"
#include "interface/CheatIO.h"
#include "ros/node_handle.h"
#include "ros/subscriber.h"
#include "simulation.h"

int main(int argc, char** argv) {
  // Get mujoco xml path
  std::string path = ros::package::getPath("hector_description");
  path += "/mjcf/hector.xml";
  std::cout << "path: " << path << std::endl;
  std::shared_ptr<Simulation> sim = std::make_shared<Simulation>(path);
  auto last = std::chrono::high_resolution_clock::now();

  // ros::init(argc, argv, "hector_control");
  // ros::NodeHandle nh;
  IOInterface* ioInter;
  ioInter = new MujocoIO("hector",sim);
  // ioInter = new CheatIO("hector",nh);
  double dt = 0.001;
  Biped biped;
  biped.setBiped();

  LegController* legController = new LegController(biped);
  std::shared_ptr<LowlevelCmd> cmd = std::make_shared<LowlevelCmd>();
  LowlevelState* state = new LowlevelState();

  std::cout << "start setup " << std::endl;
  StateEstimate stateEstimate;
  StateEstimatorContainer* stateEstimator = new StateEstimatorContainer(state, legController->data, &stateEstimate);

  stateEstimator->addEstimator<CheaterOrientationEstimator>();
  stateEstimator->addEstimator<CheaterPositionVelocityEstimator>();

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

  // main loop
  while (!glfwWindowShouldClose(sim->window)) {
    _FSMController->run();
    // sim->Step();
    // // get current time
    // auto now = std::chrono::high_resolution_clock::now();
    // // get time difference
    // auto time = std::chrono::duration_cast<std::chrono::microseconds>(now - last);
    // last = now;
    // std::cout << "time: " << time.count() << std::endl;
  }

  return 0;
}