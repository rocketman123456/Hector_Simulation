/*!
 * @file PositionVelocityEstimator.h
 * @brief compute body position/velocity in world/body frames
 */

#ifndef PROJECT_POSITIONVELOCITYESTIMATOR_H
#define PROJECT_POSITIONVELOCITYESTIMATOR_H
#include "StateEstimatorContainer.h"

class CheaterPositionVelocityEstimator : public GenericEstimator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CheaterPositionVelocityEstimator() {} 
  virtual void run ();
  virtual void setup(){
    setup_lk();
  };
  void run_lk();
  void setup_lk();

private:
  int switch_count{0};
  Biped biped;
  Eigen::Matrix<double, 12, 1> _xhat;
  Eigen::Matrix<double, 6, 1> _ps;
  Eigen::Matrix<double, 6, 1> _vs;
  Eigen::Matrix<double, 12, 12> _A;
  Eigen::Matrix<double, 12, 12> _Q0;
  Eigen::Matrix<double, 12, 12> _P;
  Eigen::Matrix<double, 14, 14> _R0;
  Eigen::Matrix<double, 12, 3> _B;
  Eigen::Matrix<double, 14, 12> _C;
};

class LinearKFPositionVelocityEstimator : public GenericEstimator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LinearKFPositionVelocityEstimator() {}
  virtual void run();
  virtual void setup();

private:
  Eigen::Matrix<double, 12, 1> _xhat;
  Eigen::Matrix<double, 6, 1> _ps;
  Eigen::Matrix<double, 6, 1> _vs;
  Eigen::Matrix<double, 12, 12> _A;
  Eigen::Matrix<double, 12, 12> _Q0;
  Eigen::Matrix<double, 12, 12> _P;
  Eigen::Matrix<double, 14, 14> _R0;
  Eigen::Matrix<double, 12, 3> _B;
  Eigen::Matrix<double, 14, 12> _C;
};

#endif