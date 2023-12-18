#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <GLFW/glfw3.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include <ros/package.h>
#include <iostream>


class Simulation {
 public:
  Simulation(const std::string& xml_path);
  ~Simulation();
  void Step(Eigen::VectorXd tau);
  mjData* data;
  mjModel* model;
  mjvCamera cam;
  mjvOption opt;
  mjvScene scene;
  mjrContext context;
  GLFWwindow* window;
  bool button_left{false}, button_middle{false}, button_right{false};
  double lastx{0}, lasty{0};
  int counter{0};
  int viewport_width{0}, viewport_height{0};
  Eigen::VectorXd qpos{Eigen::VectorXd::Zero(10)};
  Eigen::VectorXd qvel{Eigen::VectorXd::Zero(10)};
  Eigen::Vector3d bodyPos{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bodyVel{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bodyGyro{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond bodyQuat{Eigen::Quaterniond::Identity()};

  // mujoco::GlfwAdapter adapter_;
};
