#include "../../include/common/PositionVelocityEstimator.h"
#include "chrono"
#include "common/cppTypes.h"

void CheaterPositionVelocityEstimator::run() {
  if(switch_count<500){
    for (int i = 0; i < 3; i++) {
      this->_stateEstimatorData.result->position[i] =
          this->_stateEstimatorData.lowState->position[i];
      this->_stateEstimatorData.result->vWorld[i] =
          this->_stateEstimatorData.lowState->vWorld[i];
    }

    this->_stateEstimatorData.result->vBody =
        this->_stateEstimatorData.result->rBody *
        this->_stateEstimatorData.result->vWorld;
      // print out the position and velocity of the robot
    std::cout << "Cheater: ";
    std::cout << "  position: "
              << this->_stateEstimatorData.result->position.transpose();
    std::cout << "  vWorld: "
              << this->_stateEstimatorData.result->vWorld.transpose()
              << std::endl;
  } 
  run_lk();
  // std::cout<<"count number: "<<switch_count<<std::endl;
  switch_count++;
  // record time elapsed
  // std::chrono::steady_clock::time_point begin =
      // std::chrono::steady_clock::now();
  
  // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  // std::cout << "Time difference = "
  //           << std::chrono::duration_cast<std::chrono::microseconds>(end -
  //                                                                    begin)
  //                  .count()
  //           << "[µs]" << std::endl;
  // std::cout << "run StateEstimator" << std::endl;



}

void CheaterPositionVelocityEstimator::setup_lk() {
  double dt = 0.001;
  _xhat.setZero();
  _ps.setZero();
  _vs.setZero();
  _A.setZero();
  _A.block(0, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
  _A.block(0, 3, 3, 3) = dt * Eigen::Matrix<double, 3, 3>::Identity();
  _A.block(3, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
  _A.block(6, 6, 6, 6) = Eigen::Matrix<double, 6, 6>::Identity();
  _B.setZero();
  _B.block(3, 0, 3, 3) = dt * Eigen::Matrix<double, 3, 3>::Identity();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
  C1 << Eigen::Matrix<double, 3, 3>::Identity(),
      Eigen::Matrix<double, 3, 3>::Zero();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
  C2 << Eigen::Matrix<double, 3, 3>::Zero(),
      Eigen::Matrix<double, 3, 3>::Identity();
  _C.setZero();
  _C.block(0, 0, 3, 6) = C1;
  _C.block(3, 0, 3, 6) = C1;
  _C.block(6, 0, 3, 6) = C2;
  _C.block(9, 0, 3, 6) = C2;
  _C.block(0, 6, 6, 6) = double(-1) * Eigen::Matrix<double, 6, 6>::Identity();
  _C(13, 11) = double(1);
  _C(12, 8) = double(1);
  _P.setIdentity();
  _P = double(100) * _P;
  _Q0.setIdentity();
  _Q0.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<double, 3, 3>::Identity();
  _Q0.block(3, 3, 3, 3) =
      (dt * 9.8f / 20.f) * Eigen::Matrix<double, 3, 3>::Identity();
  _Q0.block(6, 6, 6, 6) = dt * Eigen::Matrix<double, 6, 6>::Identity();
  _R0.setIdentity();
}
void CheaterPositionVelocityEstimator::run_lk() {
  double process_noise_pimu = 0.5;
  double process_noise_vimu = 0.5;
  double process_noise_pfoot = 0.002;
  double sensor_noise_pimu_rel_foot = 0.001;
  double sensor_noise_vimu_rel_foot = 0.01;
  double sensor_noise_zfoot = 0.001;

  Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double, 12, 12>::Identity();
  Q.block(0, 0, 3, 3) = _Q0.block(0, 0, 3, 3) * process_noise_pimu;
  Q.block(3, 3, 3, 3) = _Q0.block(3, 3, 3, 3) * process_noise_vimu;
  Q.block(6, 6, 6, 6) = _Q0.block(6, 6, 12, 12) * process_noise_pfoot;

  Eigen::Matrix<double, 14, 14> R = Eigen::Matrix<double, 14, 14>::Identity();
  R.block(0, 0, 6, 6) = _R0.block(0, 0, 6, 6) * sensor_noise_pimu_rel_foot;
  R.block(6, 6, 6, 6) = _R0.block(6, 6, 6, 6) * sensor_noise_vimu_rel_foot;
  R.block(12, 12, 2, 2) = _R0.block(12, 12, 2, 2) * sensor_noise_zfoot;

  int qindex = 0;
  int rindex1 = 0;
  int rindex2 = 0;
  int rindex3 = 0;

  Vec3<double> g(0, 0, double(-9.81));
  Mat3<double> Rbod = this->_stateEstimatorData.result->rBody.transpose();
  // std::cout << "Rbod\n" << Rbod << "\n";
  // in old code, Rbod * se_acc + g
  Vec3<double> a = this->_stateEstimatorData.result->aWorld;
  // std::cout << "A WORLD\n" << a << "\n";
  Vec4<double> pzs = Vec4<double>::Zero();
  Vec4<double> trusts = Vec4<double>::Zero();
  Vec3<double> p0, v0;
  p0 << _xhat[0], _xhat[1], _xhat[2];
  v0 << _xhat[3], _xhat[4], _xhat[5];

  for (int i = 0; i < 2; i++) {
    int i1 = 3 * i;
    
    Vec3<double> ph = biped.getHip2Location(i); // hip positions relative to CoM
    // hw_i->leg_controller->leg_datas[i].p;
    Vec3<double> p_rel = ph + this->_stateEstimatorData.legControllerData[i].p;
    std::cout<<"p_rel: "<<p_rel.transpose()<<std::endl;
    // hw_i->leg_controller->leg_datas[i].v;
    Vec3<double> dp_rel = this->_stateEstimatorData.legControllerData[i].v;
    Vec3<double> p_f = Rbod * p_rel;
    Vec3<double> dp_f =
        Rbod *
        (this->_stateEstimatorData.result->omegaBody.cross(p_rel) + dp_rel);

    qindex = 6 + i1;
    rindex1 = i1;
    rindex2 = 6 + i1;
    rindex3 = 12 + i;

    double trust = double(1);
    double phase =
        fmin(this->_stateEstimatorData.result->contactEstimate(i), double(1));
    // std::cout << "phase " << i << ": " << phase << std::endl;
    // double trust_window = double(0.25);
    double trust_window = double(0.2);

    if (phase < trust_window) {
      trust = phase / trust_window;
    } else if (phase > (double(1) - trust_window)) {
      trust = (double(1) - phase) / trust_window;
    }
    // double high_suspect_number(1000);
    double high_suspect_number(100);

    printf("Trust %d: %.3f\n", i, trust);
    Q.block(qindex, qindex, 3, 3) =
        (double(1) + (double(1) - trust) * high_suspect_number) *
        Q.block(qindex, qindex, 3, 3);
    R.block(rindex1, rindex1, 3, 3) = 1 * R.block(rindex1, rindex1, 3, 3);
    R.block(rindex2, rindex2, 3, 3) =
        (double(1) + (double(1) - trust) * high_suspect_number) *
        R.block(rindex2, rindex2, 3, 3);
    R(rindex3, rindex3) =
        (double(1) + (double(1) - trust) * high_suspect_number) *
        R(rindex3, rindex3);

    trusts(i) = trust;

    _ps.segment(i1, 3) = -p_f;
    _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f);
    pzs(i) = (1.0f - trust) * (p0(2) + p_f(2));
  }

  Eigen::Matrix<double, 14, 1> y;
  y << _ps, _vs, pzs;
  _xhat = _A * _xhat + _B * a;
  Eigen::Matrix<double, 12, 12> At = _A.transpose();
  Eigen::Matrix<double, 12, 12> Pm = _A * _P * At + Q;
  Eigen::Matrix<double, 12, 14> Ct = _C.transpose();
  Eigen::Matrix<double, 14, 1> yModel = _C * _xhat;
  Eigen::Matrix<double, 14, 1> ey = y - yModel;
  Eigen::Matrix<double, 14, 14> S = _C * Pm * Ct + R;

  // todo compute LU only once
  Eigen::Matrix<double, 14, 1> S_ey = S.lu().solve(ey);
  _xhat += Pm * Ct * S_ey;

  Eigen::Matrix<double, 14, 12> S_C = S.lu().solve(_C);
  _P = (Eigen::Matrix<double, 12, 12>::Identity() - Pm * Ct * S_C) * Pm;

  Eigen::Matrix<double, 12, 12> Pt = _P.transpose();
  _P = (_P + Pt) / double(2);

  if (_P.block(0, 0, 2, 2).determinant() > double(0.000001)) {
    _P.block(0, 2, 2, 10).setZero();
    _P.block(2, 0, 10, 2).setZero();
    _P.block(0, 0, 2, 2) /= double(10);
  }

  // print out the position and velocity of the robot
  std::cout << "LK filter: ";
  std::cout << "  position: " << _xhat.block(0, 0, 3, 1).transpose()+Vec3<double>(0, 0, 0.047).transpose();
  std::cout << "  vWorld: " << _xhat.block(3, 0, 3, 1).transpose() << std::endl;
  if(switch_count>=500){
    this->_stateEstimatorData.result->position = _xhat.block(0, 0, 3, 1);
    this->_stateEstimatorData.result->position += Vec3<double>(0, 0, 0.047); 
    this->_stateEstimatorData.result->vWorld = _xhat.block(3, 0, 3, 1);
    this->_stateEstimatorData.result->vBody =
        this->_stateEstimatorData.result->rBody *
        this->_stateEstimatorData.result->vWorld;
  }
  
}
void LinearKFPositionVelocityEstimator::setup() {
  double dt = 0.001;
  _xhat.setZero();
  _ps.setZero();
  _vs.setZero();
  _A.setZero();
  _A.block(0, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
  _A.block(0, 3, 3, 3) = dt * Eigen::Matrix<double, 3, 3>::Identity();
  _A.block(3, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
  _A.block(6, 6, 6, 6) = Eigen::Matrix<double, 6, 6>::Identity();
  _B.setZero();
  _B.block(3, 0, 3, 3) = dt * Eigen::Matrix<double, 3, 3>::Identity();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
  C1 << Eigen::Matrix<double, 3, 3>::Identity(),
      Eigen::Matrix<double, 3, 3>::Zero();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
  C2 << Eigen::Matrix<double, 3, 3>::Zero(),
      Eigen::Matrix<double, 3, 3>::Identity();
  _C.setZero();
  _C.block(0, 0, 3, 6) = C1;
  _C.block(3, 0, 3, 6) = C1;
  _C.block(6, 0, 3, 6) = C2;
  _C.block(9, 0, 3, 6) = C2;
  _C.block(0, 6, 6, 6) = double(-1) * Eigen::Matrix<double, 6, 6>::Identity();
  _C(13, 11) = double(1);
  _C(12, 8) = double(1);
  _P.setIdentity();
  _P = double(100) * _P;
  _Q0.setIdentity();
  _Q0.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<double, 3, 3>::Identity();
  _Q0.block(3, 3, 3, 3) =
      (dt * 9.8f / 20.f) * Eigen::Matrix<double, 3, 3>::Identity();
  _Q0.block(6, 6, 6, 6) = dt * Eigen::Matrix<double, 6, 6>::Identity();
  _R0.setIdentity();
}

void LinearKFPositionVelocityEstimator::run() {
  double process_noise_pimu = 0.2;
  double process_noise_vimu = 0.2;
  double process_noise_pfoot = 0.002;
  double sensor_noise_pimu_rel_foot = 0.005;
  double sensor_noise_vimu_rel_foot = 0.1;
  double sensor_noise_zfoot = 0.005;

  Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double, 12, 12>::Identity();
  Q.block(0, 0, 3, 3) = _Q0.block(0, 0, 3, 3) * process_noise_pimu;
  Q.block(3, 3, 3, 3) = _Q0.block(3, 3, 3, 3) * process_noise_vimu;
  Q.block(6, 6, 6, 6) = _Q0.block(6, 6, 12, 12) * process_noise_pfoot;

  Eigen::Matrix<double, 14, 14> R = Eigen::Matrix<double, 14, 14>::Identity();
  R.block(0, 0, 6, 6) = _R0.block(0, 0, 6, 6) * sensor_noise_pimu_rel_foot;
  R.block(6, 6, 6, 6) = _R0.block(6, 6, 6, 6) * sensor_noise_vimu_rel_foot;
  R.block(12, 12, 2, 2) = _R0.block(12, 12, 2, 2) * sensor_noise_zfoot;

  int qindex = 0;
  int rindex1 = 0;
  int rindex2 = 0;
  int rindex3 = 0;

  Vec3<double> g(0, 0, double(-9.81));
  Mat3<double> Rbod = this->_stateEstimatorData.result->rBody.transpose();
  // in old code, Rbod * se_acc + g
  Vec3<double> a = this->_stateEstimatorData.result->aWorld + g;
  // std::cout << "A WORLD\n" << a << "\n";
  Vec4<double> pzs = Vec4<double>::Zero();
  Vec4<double> trusts = Vec4<double>::Zero();
  Vec3<double> p0, v0;
  p0 << _xhat[0], _xhat[1], _xhat[2];
  v0 << _xhat[3], _xhat[4], _xhat[5];

  for (int i = 0; i < 2; i++) {
    int i1 = 3 * i;
    Biped biped;
    Vec3<double> ph = biped.getHip2Location(i); // hip positions relative to CoM
    // hw_i->leg_controller->leg_datas[i].p;
    Vec3<double> p_rel = ph + this->_stateEstimatorData.legControllerData[i].p;
    // hw_i->leg_controller->leg_datas[i].v;
    Vec3<double> dp_rel = this->_stateEstimatorData.legControllerData[i].v;
    Vec3<double> p_f = Rbod * p_rel;
    Vec3<double> dp_f =
        Rbod *
        (this->_stateEstimatorData.result->omegaBody.cross(p_rel) + dp_rel);

    qindex = 6 + i1;
    rindex1 = i1;
    rindex2 = 6 + i1;
    rindex3 = 12 + i;

    double trust = double(1);
    double phase =
        fmin(this->_stateEstimatorData.result->contactEstimate(i), double(1));
    std::cout << "phase " << i << ": " << phase << std::endl;
    // double trust_window = double(0.25);
    double trust_window = double(0.2);

    if (phase < trust_window) {
      trust = phase / trust_window;
    } else if (phase > (double(1) - trust_window)) {
      trust = (double(1) - phase) / trust_window;
    }
    // double high_suspect_number(1000);
    double high_suspect_number(100);

    // printf("Trust %d: %.3f\n", i, trust);
    Q.block(qindex, qindex, 3, 3) =
        (double(1) + (double(1) - trust) * high_suspect_number) *
        Q.block(qindex, qindex, 3, 3);
    R.block(rindex1, rindex1, 3, 3) = 1 * R.block(rindex1, rindex1, 3, 3);
    R.block(rindex2, rindex2, 3, 3) =
        (double(1) + (double(1) - trust) * high_suspect_number) *
        R.block(rindex2, rindex2, 3, 3);
    R(rindex3, rindex3) =
        (double(1) + (double(1) - trust) * high_suspect_number) *
        R(rindex3, rindex3);

    trusts(i) = trust;

    _ps.segment(i1, 3) = -p_f;
    _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f);
    pzs(i) = (1.0f - trust) * (p0(2) + p_f(2));
  }

  Eigen::Matrix<double, 14, 1> y;
  y << _ps, _vs, pzs;
  _xhat = _A * _xhat + _B * a;
  Eigen::Matrix<double, 12, 12> At = _A.transpose();
  Eigen::Matrix<double, 12, 12> Pm = _A * _P * At + Q;
  Eigen::Matrix<double, 12, 14> Ct = _C.transpose();
  Eigen::Matrix<double, 14, 1> yModel = _C * _xhat;
  Eigen::Matrix<double, 14, 1> ey = y - yModel;
  Eigen::Matrix<double, 14, 14> S = _C * Pm * Ct + R;

  // todo compute LU only once
  Eigen::Matrix<double, 14, 1> S_ey = S.lu().solve(ey);
  _xhat += Pm * Ct * S_ey;

  Eigen::Matrix<double, 14, 12> S_C = S.lu().solve(_C);
  _P = (Eigen::Matrix<double, 12, 12>::Identity() - Pm * Ct * S_C) * Pm;

  Eigen::Matrix<double, 12, 12> Pt = _P.transpose();
  _P = (_P + Pt) / double(2);

  if (_P.block(0, 0, 2, 2).determinant() > double(0.000001)) {
    _P.block(0, 2, 2, 10).setZero();
    _P.block(2, 0, 10, 2).setZero();
    _P.block(0, 0, 2, 2) /= double(10);
  }

  // print out the position and velocity of the robot
  std::cout << "LK filter: ";
  std::cout << "position: " << _xhat.block(0, 0, 3, 1);
  std::cout << "vWorld: " << _xhat.block(3, 0, 3, 1);
  // this->_stateEstimatorData.result->position = _xhat.block(0, 0, 3, 1);
  // this->_stateEstimatorData.result->vWorld = _xhat.block(3, 0, 3, 1);
  // this->_stateEstimatorData.result->vBody =
  //     this->_stateEstimatorData.result->rBody *
  //     this->_stateEstimatorData.result->vWorld;
}
