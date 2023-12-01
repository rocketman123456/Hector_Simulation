#include "../../include/common/LegController.h"
#include <memory>
#include <eigen3/Eigen/Core>

// upper level of joint controller 
// send data to joint controller

void LegControllerCommand::zero(){
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
void LegControllerData::zero(){
    q = Vec5<double>::Zero();
    qd = Vec5<double>::Zero();
    p = Vec3<double>::Zero();
    v = Vec3<double>::Zero();
    J_force_moment = Mat65<double>::Zero();
    J_force = Mat35<double>::Zero();
    tau = Vec5<double>::Zero();
}

void LegController::zeroCommand(){
    for (int i = 0; i < 2; i++){
        commands[i].zero();
    }
}

void LegController::updateData(const LowlevelState* state){
    for (int leg = 0; leg < 2; leg++){
        for(int j = 0; j < 5; j++){
            data[leg].q(j) = state->motorState[leg*5+j].q;
            data[leg].qd(j) = state->motorState[leg*5+j].dq;
            data[leg].tau(j) = state->motorState[leg*5+j].tauEst;
            // std::cout << "motor joint data" << leg*5+j << ": "<< data[leg].q(j) << std::endl;
        }

        computeLegJacobianAndPosition(_biped, data[leg].q, &(data[leg].J_force_moment), &(data[leg].J_force), &(data[leg].p), leg);
        data[leg].v = data[leg].J_force * data[leg].qd;
    }

}

void LegController::updateCommand(std::shared_ptr<LowlevelCmd> cmd){

    for (int i = 0; i < 2; i++){
        Vec6<double> footForce = commands[i].feedforwardForce;
        //利用雅可比，根据足端力计算关节力矩。这里忽略了腿的动力学，只考虑了静力学
        Vec5<double> legtau = data[i].J_force_moment.transpose() * footForce; // force moment from stance leg
        
        // for(int j = 0; j < 5; j++){
        //     std::cout << "legtau" << j << ": "<< legtau(j) << std::endl;
        // }

        // cartesian PD control for swing foot
        // 通过笛卡尔空间的kp和kd判断需不需要进行摆动腿力矩的计算
        if(commands[i].kpCartesian(0,0) != 0 || commands[i].kdCartesian(0,0) != 0)
        {
            // 摆动腿的足端在笛卡尔空间的力（弹簧阻尼模型）
            Vec3<double> footForce_3d = commands[i].kpCartesian * (commands[i].pDes - data[i].p) +
                                        commands[i].kdCartesian * (commands[i].vDes - data[i].v);
            // 摆动腿的足端在笛卡尔空间的力转换到关节空间的力矩，此处的J_force和上面的J_force_moment不同
            Vec5<double> swingtau = data[i].J_force.transpose() * footForce_3d ;

            // maintain hip angle tracking
            //单独控制摆动腿的髋关节，使其保持0角度
            double kphip1 = 15;
            double kdhip1 = 1;
            swingtau(0) = kphip1*(0-data[i].q(0)) + kdhip1*(0-data[i].qd(0));
            // make sure foot is parallel with the ground
            //单独控制摆动腿的脚踝关节，使其保持0角度，与地面平行
            swingtau(4) = commands[i].kptoe * (-data[i].q(3)-data[i].q(2)-data[i].q(4))+commands[i].kdtoe*(0-data[i].qd(4));

            for(int j = 0; j < 5; j++)
            {
                legtau(j) += swingtau(j);
            }
        }

        commands[i].tau += legtau;
        // 更新关节力矩
        for (int j = 0; j < 5; j++){
            cmd->motorCmd[i*5+j].tau = commands[i].tau(j);
            cmd->motorCmd[i*5+j].q = commands[i].qDes(j);
            cmd->motorCmd[i*5+j].dq = commands[i].qdDes(j);
            cmd->motorCmd[i*5+j].Kp = commands[i].kpJoint(j,j);
            cmd->motorCmd[i*5+j].Kd = commands[i].kdJoint(j,j);
            // std::cout << Side[i] << " " << limbName[j] <<" torque cmd  =  " << cmd->motorCmd[i*5+j].tau << std::endl;            
        }

        commands[i].tau << 0, 0, 0, 0, 0; // zero torque command to prevent interference
        
        
   
    }
    //std::cout << "cmd sent" << std::endl;
   
}
void computeLegJacobianAndPosition(Biped& _biped, Vec5<double>& q, Mat65<double>* J_f_m, Mat35<double>* J_f, 
                                       Vec3<double>* p, int leg)
{
    //为什么会有两个雅可比矩阵？
    //J_f_m维度是6*5，J_f维度是3*5，J_f_m标准的雅可比矩阵，J_f是J_f_m的前三列
    //对于摆动腿，不需要对脚掌的姿态进行控制，所以只需要前三列
    q(2) = q(2) + 0.3*3.14159;
    q(3) = q(3) - 0.6*3.14159;
    q(4) = q(4) + 0.3*3.14159;

    double q0 = q(0);
    double q1 = q(1);
    double q2 = q(2);
    double q3 = q(3);
    double q4 = q(4);

    
    double side = -1.0; // 1 for Left legs; -1 for right legs
    if (leg == 0){
        // std::cout<< "Leg Sign checked" << std::endl;
        side = 1.0;
    }
    // std::cout<< "Leg Sign" << side << std::endl;
    
    //如果指针不为nullptr,则计算雅克比
    if(J_f_m){
    J_f_m->operator()(0, 0) =  sin(q0)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2) + 0.0135) + cos(q0)*(0.015*side + cos(q1)*(0.018*side + 0.0025) - 1.0*sin(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)));
    J_f_m->operator()(1, 0) =  sin(q0)*(0.015*side + cos(q1)*(0.018*side + 0.0025) - 1.0*sin(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2))) - 1.0*cos(q0)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2) + 0.0135);
    J_f_m->operator()(2, 0) =  0.0;
    J_f_m->operator()(3, 0) = 0.0;
    J_f_m->operator()(4, 0) = 0.0;
    J_f_m->operator()(5, 0) = 1.0;

    J_f_m->operator()(0, 1) =  -1.0*sin(q0)*(sin(q1)*(0.018*side + 0.0025) + cos(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)));
    J_f_m->operator()(1, 1) =  cos(q0)*(sin(q1)*(0.018*side + 0.0025) + cos(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)));
    J_f_m->operator()(2, 1) =  sin(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)) - 1.0*cos(q1)*(0.018*side + 0.0025);
    J_f_m->operator()(3, 1) = cos(q0);
    J_f_m->operator()(4, 1) = sin(q0);
    J_f_m->operator()(5, 1) = 0.0;

    J_f_m->operator()(0, 2) =  sin(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2)) - 1.0*cos(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2));
    J_f_m->operator()(1, 2) =  - 1.0*sin(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)) - 1.0*cos(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2));
    J_f_m->operator()(2, 2) =  cos(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2));
    J_f_m->operator()(3, 2) = -cos(q1)*sin(q0);
    J_f_m->operator()(4, 2) = cos(q0)*cos(q1);
    J_f_m->operator()(5, 2) = sin(q1);

    J_f_m->operator()(0, 3) =  sin(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3)) - 1.0*cos(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3));
    J_f_m->operator()(1, 3) =  - 1.0*sin(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3)) - 1.0*cos(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3));
    J_f_m->operator()(2, 3) =  cos(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3));
    J_f_m->operator()(3, 3) = -cos(q1)*sin(q0);
    J_f_m->operator()(4, 3) = cos(q0)*cos(q1);
    J_f_m->operator()(5, 3) = sin(q1);

    J_f_m->operator()(0, 4) =  0.04*sin(q2 + q3 + q4)*sin(q0)*sin(q1) - 0.04*cos(q2 + q3 + q4)*cos(q0);
    J_f_m->operator()(1, 4) =  - 0.04*cos(q2 + q3 + q4)*sin(q0) - 0.04*sin(q2 + q3 + q4)*cos(q0)*sin(q1);
    J_f_m->operator()(2, 4) =  0.04*sin(q2 + q3 + q4)*cos(q1);
    J_f_m->operator()(3, 4) = -cos(q1)*sin(q0);
    J_f_m->operator()(4, 4) = cos(q0)*cos(q1);
    J_f_m->operator()(5, 4) = sin(q1);
   }

   if(J_f){
    J_f->operator()(0, 0) =  sin(q0)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2) + 0.0135) + cos(q0)*(0.015*side + cos(q1)*(0.018*side + 0.0025) - 1.0*sin(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)));
    J_f->operator()(1, 0) =  sin(q0)*(0.015*side + cos(q1)*(0.018*side + 0.0025) - 1.0*sin(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2))) - 1.0*cos(q0)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2) + 0.0135);
    J_f->operator()(2, 0) =  0.0;

    J_f->operator()(0, 1) =  -1.0*sin(q0)*(sin(q1)*(0.018*side + 0.0025) + cos(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)));
    J_f->operator()(1, 1) =  cos(q0)*(sin(q1)*(0.018*side + 0.0025) + cos(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)));
    J_f->operator()(2, 1) =  sin(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)) - 1.0*cos(q1)*(0.018*side + 0.0025);

    J_f->operator()(0, 2) =  sin(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2)) - 1.0*cos(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2));
    J_f->operator()(1, 2) =  - 1.0*sin(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)) - 1.0*cos(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2));
    J_f->operator()(2, 2) =  cos(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2));

    J_f->operator()(0, 3) =  sin(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3)) - 1.0*cos(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3));
    J_f->operator()(1, 3) =  - 1.0*sin(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3)) - 1.0*cos(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3));
    J_f->operator()(2, 3) =  cos(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3));

    J_f->operator()(0, 4) =  0.04*sin(q2 + q3 + q4)*sin(q0)*sin(q1) - 0.04*cos(q2 + q3 + q4)*cos(q0);
    J_f->operator()(1, 4) =  - 0.04*cos(q2 + q3 + q4)*sin(q0) - 0.04*sin(q2 + q3 + q4)*cos(q0)*sin(q1);
    J_f->operator()(2, 4) =  0.04*sin(q2 + q3 + q4)*cos(q1);
    
    }

   if(p){
    p->operator()(0) = - (3*cos(q0))/200 - (9*sin(q4)*(cos(q3)*(cos(q0)*cos(q2) - sin(q0)*sin(q1)*sin(q2)) - sin(q3)*(cos(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1))))/250 - (11*cos(q0)*sin(q2))/50 - ( (side)*sin(q0))/50 - (11*cos(q3)*(cos(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)))/50 - (11*sin(q3)*(cos(q0)*cos(q2) - sin(q0)*sin(q1)*sin(q2)))/50 - (9*cos(q4)*(cos(q3)*(cos(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)) + sin(q3)*(cos(q0)*cos(q2) - sin(q0)*sin(q1)*sin(q2))))/250 - (23*cos(q1)* (side)*sin(q0))/1000 - (11*cos(q2)*sin(q0)*sin(q1))/50;
    p->operator()(1) = (cos(q0)* (side))/50 - (9*sin(q4)*(cos(q3)*(cos(q2)*sin(q0) + cos(q0)*sin(q1)*sin(q2)) - sin(q3)*(sin(q0)*sin(q2) - cos(q0)*cos(q2)*sin(q1))))/250 - (3*sin(q0))/200 - (11*sin(q0)*sin(q2))/50 - (11*cos(q3)*(sin(q0)*sin(q2) - cos(q0)*cos(q2)*sin(q1)))/50 - (11*sin(q3)*(cos(q2)*sin(q0) + cos(q0)*sin(q1)*sin(q2)))/50 - (9*cos(q4)*(cos(q3)*(sin(q0)*sin(q2) - cos(q0)*cos(q2)*sin(q1)) + sin(q3)*(cos(q2)*sin(q0) + cos(q0)*sin(q1)*sin(q2))))/250 + (23*cos(q0)*cos(q1)* (side))/1000 + (11*cos(q0)*cos(q2)*sin(q1))/50;
    p->operator()(2) = (23*(side)*sin(q1))/1000 - (11*cos(q1)*cos(q2))/50 - (9*cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))/250 + (9*sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))/250 - (11*cos(q1)*cos(q2)*cos(q3))/50 + (11*cos(q1)*sin(q2)*sin(q3))/50 - 3.0/50.0;
   }
}