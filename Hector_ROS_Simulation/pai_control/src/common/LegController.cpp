#include "../../include/common/LegController.h"
#include <eigen3/Eigen/Core>
#include <memory>

// upper level of joint controller
// send data to joint controller

void LegControllerCommand::zero() {
  tau = Vec5<double>::Zero();
  qDes = Vec5<double>::Zero();
  qdDes = Vec5<double>::Zero();
  pDes = Vec3<double>::Zero();
  vDes = Vec3<double>::Zero();
  feedforwardForce = Vec6<double>::Zero();
  hiptoeforce = Vec3<double>::Zero();
  kpCartesian = Mat3<double>::Zero();
  kdCartesian = Mat3<double>::Zero();
  kpJoint = Mat5<double>::Zero();
  kdJoint = Mat5<double>::Zero();
  double kptoe = 0;
  double kdtoe = 0;
}

/*!
 * Zero leg data
 */
void LegControllerData::zero() {
  q = Vec5<double>::Zero();
  qd = Vec5<double>::Zero();
  p = Vec3<double>::Zero();
  v = Vec3<double>::Zero();
  J_force_moment = Mat65<double>::Zero();
  J_force = Mat35<double>::Zero();
  tau = Vec5<double>::Zero();
}

void LegController::zeroCommand() {
  for (int i = 0; i < 2; i++) {
    commands[i].zero();
  }
}

void LegController::updateData(const LowlevelState *state) {
  for (int leg = 0; leg < 2; leg++) {
    for (int j = 0; j < 5; j++) {
      data[leg].q(j) = state->motorState[leg * 5 + j].q;
      data[leg].qd(j) = state->motorState[leg * 5 + j].dq;
      data[leg].tau(j) = state->motorState[leg * 5 + j].tauEst;
      // std::cout << "motor joint data" << leg*5+j << ": "<< data[leg].q(j) <<
      // std::endl;
    }

    computeLegJacobianAndPosition(_biped, data[leg].q,
                                  &(data[leg].J_force_moment),
                                  &(data[leg].J_force), &(data[leg].p), leg);
    data[leg].v = data[leg].J_force * data[leg].qd;
  }
}

void LegController::updateCommand(std::shared_ptr<LowlevelCmd> cmd) {

  for (int i = 0; i < 2; i++) {
    Vec6<double> footForce = commands[i].feedforwardForce;
    // 利用雅可比，根据足端力计算关节力矩。这里忽略了腿的动力学，只考虑了静力学
    Vec5<double> legtau = data[i].J_force_moment.transpose() *
                          footForce; // force moment from stance leg
    // Vec5<double> legtau;
    // legtau.setZero();

    // cartesian PD control for swing foot
    // 通过笛卡尔空间的kp和kd判断需不需要进行摆动腿力矩的计算
    if (commands[i].kpCartesian(0, 0) != 0 ||
        commands[i].kdCartesian(0, 0) != 0) {
      // 摆动腿的足端在笛卡尔空间的力（弹簧阻尼模型）
      Vec3<double> footForce_3d =
          commands[i].kpCartesian * (commands[i].pDes - data[i].p) +
          commands[i].kdCartesian * (commands[i].vDes - data[i].v);
      // // print pDes
      // std::cout << "pDes: " << commands[i].pDes.transpose() << std::endl;
      // std::cout << "p: " << data[i].p.transpose() << std::endl;
      // 摆动腿的足端在笛卡尔空间的力转换到关节空间的力矩，此处的J_force和上面的J_force_moment不同
      Vec5<double> swingtau = data[i].J_force.transpose() * footForce_3d;

      // maintain hip angle tracking
      // 单独控制摆动腿的髋关节，使其保持0角度
      double kphip1 = 0.5;
      double kdhip1 = 0.1;
      swingtau(1) = kphip1 * (0 - data[i].q(1));
      // make sure foot is parallel with the ground
      // 单独控制摆动腿的脚踝关节，使其保持0角度，与地面平行
      swingtau(4) =
          commands[i].kptoe * (- data[i].q(3) - data[i].q(2) - data[i].q(4));
          // commands[i].kdtoe * (0 - data[i].qd(4));
      //print q
      // std::cout << "q: " << data[i].q.transpose() << std::endl;

      for (int j = 0; j < 5; j++) {
        legtau(j) += swingtau(j);
      }
    }

    commands[i].tau += legtau;
    // 更新关节力矩
    for (int j = 0; j < 5; j++) {
      cmd->motorCmd[i * 5 + j].tau = commands[i].tau(j);
      cmd->motorCmd[i * 5 + j].q = commands[i].qDes(j);
      cmd->motorCmd[i * 5 + j].dq = commands[i].qdDes(j);
      cmd->motorCmd[i * 5 + j].Kp = commands[i].kpJoint(j, j);
      cmd->motorCmd[i * 5 + j].Kd = commands[i].kdJoint(j, j);
      // std::cout << Side[i] << " " << limbName[j] <<" torque cmd  =  " <<
      // cmd->motorCmd[i*5+j].tau << std::endl;
    }

    commands[i].tau << 0, 0, 0, 0,
        0; // zero torque command to prevent interference
  }
  // std::cout << "cmd sent" << std::endl;
}
void computeLegJacobianAndPosition(Biped &_biped, Vec5<double> &q,
                                   Mat65<double> *J_f_m, Mat35<double> *J_f,
                                   Vec3<double> *p, int leg) {
  // 为什么会有两个雅可比矩阵？
  // J_f_m维度是6*5，J_f维度是3*5，J_f_m标准的雅可比矩阵，J_f是J_f_m的前三列
  // 对于摆动腿，不需要对脚掌的姿态进行控制，所以只需要前三列
  Eigen::VectorXd q_full;
  int foot_l_id = 5;
  int foot_r_id = 10;
  q_full.setZero(10);

  pinocchio::Data::Matrix6x J(6, _biped.model_.nv);
  J.setZero();
  pinocchio::Data::Matrix6x J_leg(6, 5);
  J_leg.setZero();

  if (leg == 0) {
    q_full.head(5) = q;
  } else {
    q_full.tail(5) = q;
  }
  // std::cout<<"q full:"<<q_full.transpose()<<std::endl;
  pinocchio::forwardKinematics(_biped.model_, _biped.data_, q_full);
  pinocchio::updateFramePlacements(_biped.model_, _biped.data_);
  Vec3<double> pos_foot;
  pos_foot.setZero();

  if (leg == 0) {
    pos_foot = _biped.data_.oMi[foot_l_id].translation();
    auto ori_foot = _biped.data_.oMi[foot_l_id].rotation();
    pinocchio::Data::Matrix6x R_leg(6, 6);
    R_leg.setZero();
    R_leg.topLeftCorner(3, 3) = ori_foot;
    R_leg.bottomRightCorner(3, 3) = ori_foot;
    pinocchio::computeJointJacobian(_biped.model_, _biped.data_, q_full,
                                    foot_l_id, J);
    J_leg = J.leftCols(5);
    J_leg = R_leg * J_leg;
  } else {
    pos_foot = _biped.data_.oMi[foot_r_id].translation();
    auto ori_foot = _biped.data_.oMi[foot_r_id].rotation();
    pinocchio::Data::Matrix6x R_leg(6, 6);
    R_leg.setZero();
    R_leg.topLeftCorner(3, 3) = ori_foot;
    R_leg.bottomRightCorner(3, 3) = ori_foot;
    pinocchio::computeJointJacobian(_biped.model_, _biped.data_, q_full,
                                    foot_r_id, J);
    J_leg = J.rightCols(5);
    J_leg = R_leg * J_leg;
  }


  // 如果指针不为nullptr,则计算雅克比
  if (J_f_m) {
    *J_f_m = J_leg;
  }

  if (J_f) {
    *J_f = J_leg.topRows(3);
  }

  if (p) {
    if (leg == 0) {
      // std::cout<<"pinocchio pos:"<<std::endl;
      p->operator()(0) = pos_foot(0) - 0.049;
      p->operator()(1) = pos_foot(1) - 0.075;
      p->operator()(2) = pos_foot(2) + 0.02;
      // std::cout<<p->operator()(0)<<" "<<p->operator()(1)<<"
      // "<<p->operator()(2)<<std::endl;
    } else {
      // std::cout<<"pinocchio pos:"<<std::endl;
      p->operator()(0) = pos_foot(0) - 0.049;
      p->operator()(1) = pos_foot(1) + 0.075;
      p->operator()(2) = pos_foot(2) + 0.02;
      // std::cout<<p->operator()(0)<<" "<<p->operator()(1)<<"
      // "<<p->operator()(2)<<std::endl;
    }
  }
}