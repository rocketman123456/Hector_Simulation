#include "interface/MujocoIO.h"

#include <Eigen/Core>
#include <memory>

#include "Eigen/src/Core/Matrix.h"
#include "messages/LowLevelCmd.h"
#include "messages/LowlevelState.h"

void MujocoIO::sendRecv(const std::shared_ptr<LowlevelCmd> cmd, LowlevelState* state) {
  // set command torque
  Eigen::VectorXd jointTorque(10);
  for (int i = 0; i < 10; i++) {
    jointTorque(i) = cmd->motorCmd[i].tau;
  }
  // sim
  sim_->Step(jointTorque);

  // return state 
  for (int i = 0; i < 10; i++) {
    state->motorState[i].q = sim_->qpos(i);
    state->motorState[i].dq = sim_->qvel(i);
    state->motorState[i].tauEst = jointTorque(i);
  }
  for (int i = 0; i < 3; i++) {
    state->imu.gyroscope[i] = sim_->bodyGyro(i);
    state->position[i] = sim_->bodyPos(i);
    state->vWorld[i] = sim_->bodyVel(i);
  }
  state->imu.quaternion[0] = sim_->bodyQuat.w();
  state->imu.quaternion[1] = sim_->bodyQuat.x();
  state->imu.quaternion[2] = sim_->bodyQuat.y();
  state->imu.quaternion[3] = sim_->bodyQuat.z();

  //update command vel
  cmdPanel->updateVelCmd(state);
  state->userCmd = cmdPanel->getUserCmd();
  state->userValue = cmdPanel->getUserValue();
}