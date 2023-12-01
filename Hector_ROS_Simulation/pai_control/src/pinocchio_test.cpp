#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include <ros/package.h>

int main() {
  std::string path = ros::package::getPath("hector_description");
  path += "/xacro/hector.urdf";

  pinocchio::Model model_;
  pinocchio::urdf::buildModel(path, model_);
  pinocchio::Data data_(model_);
  std::cout<<model_.name<<std::endl;
  //print all the joint names of model_
  Eigen::VectorXd q = pinocchio::neutral(model_);
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);

  return 0;
}