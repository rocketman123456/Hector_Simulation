#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include <iostream>
#include <ros/package.h>

int main() {
  // std::string path = ros::package::getPath("hector_description");
  // path += "/xacro/hector.urdf";
  std::string path = ros::package::getPath("pai_description");
  path += "/xacro/pai.urdf";

  pinocchio::Model model_;
  pinocchio::urdf::buildModel(path, model_);
  pinocchio::Data data_(model_);
  std::cout << model_.name << std::endl;

  Eigen::VectorXd q = pinocchio::neutral(model_);
  std::cout << "q: " << q.transpose() << std::endl;
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  // print all the joint names of model_
  for (pinocchio::JointIndex joint_id = 0;
       joint_id < (pinocchio::JointIndex)model_.njoints; ++joint_id)
    std::cout << std::setw(24) << std::left << model_.names[joint_id] << ": "
              << std::fixed << std::setprecision(4)
              << data_.oMi[joint_id].translation().transpose() << std::endl;

  // // 测试hector左腿正运动学
  // double q0 = q(0);
  // double q1 = q(1);
  // double q2 = q(2);
  // double q3 = q(3);
  // double q4 = q(4);

  // double side = 1.0; // 1 for Left legs; -1 for right legs
  // double x =
  //     -(3 * cos(q0)) / 200 -
  //     (9 * sin(q4) *
  //      (cos(q3) * (cos(q0) * cos(q2) - sin(q0) * sin(q1) * sin(q2)) -
  //       sin(q3) * (cos(q0) * sin(q2) + cos(q2) * sin(q0) * sin(q1)))) /
  //         250 -
  //     (11 * cos(q0) * sin(q2)) / 50 - ((side)*sin(q0)) / 50 -
  //     (11 * cos(q3) * (cos(q0) * sin(q2) + cos(q2) * sin(q0) * sin(q1))) / 50 -
  //     (11 * sin(q3) * (cos(q0) * cos(q2) - sin(q0) * sin(q1) * sin(q2))) / 50 -
  //     (9 * cos(q4) *
  //      (cos(q3) * (cos(q0) * sin(q2) + cos(q2) * sin(q0) * sin(q1)) +
  //       sin(q3) * (cos(q0) * cos(q2) - sin(q0) * sin(q1) * sin(q2)))) /
  //         250 -
  //     (23 * cos(q1) * (side)*sin(q0)) / 1000 -
  //     (11 * cos(q2) * sin(q0) * sin(q1)) / 50;
  // double y =
  //     (cos(q0) * (side)) / 50 -
  //     (9 * sin(q4) *
  //      (cos(q3) * (cos(q2) * sin(q0) + cos(q0) * sin(q1) * sin(q2)) -
  //       sin(q3) * (sin(q0) * sin(q2) - cos(q0) * cos(q2) * sin(q1)))) /
  //         250 -
  //     (3 * sin(q0)) / 200 - (11 * sin(q0) * sin(q2)) / 50 -
  //     (11 * cos(q3) * (sin(q0) * sin(q2) - cos(q0) * cos(q2) * sin(q1))) / 50 -
  //     (11 * sin(q3) * (cos(q2) * sin(q0) + cos(q0) * sin(q1) * sin(q2))) / 50 -
  //     (9 * cos(q4) *
  //      (cos(q3) * (sin(q0) * sin(q2) - cos(q0) * cos(q2) * sin(q1)) +
  //       sin(q3) * (cos(q2) * sin(q0) + cos(q0) * sin(q1) * sin(q2)))) /
  //         250 +
  //     (23 * cos(q0) * cos(q1) * (side)) / 1000 +
  //     (11 * cos(q0) * cos(q2) * sin(q1)) / 50;
  // double z = (23 * (side)*sin(q1)) / 1000 - (11 * cos(q1) * cos(q2)) / 50 -
  //            (9 * cos(q4) *
  //             (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3))) /
  //                250 +
  //            (9 * sin(q4) *
  //             (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))) /
  //                250 -
  //            (11 * cos(q1) * cos(q2) * cos(q3)) / 50 +
  //            (11 * cos(q1) * sin(q2) * sin(q3)) / 50 - 3.0 / 50.0;

  // std::cout << std::fixed << std::setprecision(4) << "toe x:" << x << " y:" << y
  //           << " z:" << z << std::endl;
  return 0;
}