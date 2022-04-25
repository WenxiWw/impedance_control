#include "skillbase.h"
#include <cstdarg>
#include <math.h>

SkillBase::~SkillBase()
{
}

void SkillBase::spin_wrap()
{
    if(is_initialised)
    {
        spin();
    }
}

franka::Torques SkillBase::control_wrap(franka::RobotState st)
{
  mtx.lock();

  if(!is_initialised)
  {
    initialize(st);
    is_initialised = true;
  }

  last_state = st;

  franka::Torques cmd = control(st);

  mtx.unlock();

  return{cmd};
}

void SkillBase::wave(Flag flag)
{
    is_waving = true;
    this->flag = flag;
    is_initialised = false;
}

void SkillBase::set_modelbase(ModelBase *modelbase)
{
    this->model.set_modelbase(modelbase);
}

void SkillBase::set_gripperbase(GripperBase *gripperbase)
{
    this->gripper.set_gripperbase(gripperbase);
}


Skill_1::Skill_1()
{
    q_init << -0.0345401,-0.0373237,0.047841,-1.74195,0.000647148,1.81225,0.653404;
}

Skill_1::Skill_1(Eigen::Matrix<double, 7, 1> q_init)
{
    this->q_init = q_init;
}

void Skill_1::initialize(franka::RobotState st)
{
    jimp.p.K_theta << 2000, 2000, 2000, 2000, 100, 100, 100;
    jimp.p.D_theta.setConstant(1);
    jimp.p.enable_ffwd_acc.setZero();
    jimp.p.enable_ffwd_vel.setZero();

    set_U(st);

    jimp.u.theta_d = q_init; // 0, -M_PI/2.0, 0, -M_PI/2.0, 0, M_PI/2.0, -M_PI/2.0;
    jimp.u.dtheta_d.setZero();
    jimp.u.ddtheta_d.setZero();

    jimp.initialize();
}

franka::Torques Skill_1::control(franka::RobotState st)
{
    set_U(st);

    jimp.u.theta_d = q_init;
    jimp.u.dtheta_d.setZero();
    jimp.u.ddtheta_d.setZero();

    jimp.step();

    const double tau_max_1 = 87;
    const double tau_max_2 = 12;

    if(jimp.y.tau_J_d[0] > tau_max_1) jimp.y.tau_J_d[0] = tau_max_1;
    if(jimp.y.tau_J_d[1] > tau_max_1) jimp.y.tau_J_d[1] = tau_max_1;
    if(jimp.y.tau_J_d[2] > tau_max_1) jimp.y.tau_J_d[2] = tau_max_1;
    if(jimp.y.tau_J_d[3] > tau_max_1) jimp.y.tau_J_d[3] = tau_max_1;
    if(jimp.y.tau_J_d[4] > tau_max_2) jimp.y.tau_J_d[4] = tau_max_2;
    if(jimp.y.tau_J_d[5] > tau_max_2) jimp.y.tau_J_d[5] = tau_max_2;
    if(jimp.y.tau_J_d[6] > tau_max_2) jimp.y.tau_J_d[6] = tau_max_2;

    if(jimp.y.tau_J_d[0] < -tau_max_1) jimp.y.tau_J_d[0] = -tau_max_1;
    if(jimp.y.tau_J_d[1] < -tau_max_1) jimp.y.tau_J_d[1] = -tau_max_1;
    if(jimp.y.tau_J_d[2] < -tau_max_1) jimp.y.tau_J_d[2] = -tau_max_1;
    if(jimp.y.tau_J_d[3] < -tau_max_1) jimp.y.tau_J_d[3] = -tau_max_1;
    if(jimp.y.tau_J_d[4] < -tau_max_2) jimp.y.tau_J_d[4] = -tau_max_2;
    if(jimp.y.tau_J_d[5] < -tau_max_2) jimp.y.tau_J_d[5] = -tau_max_2;
    if(jimp.y.tau_J_d[6] < -tau_max_2) jimp.y.tau_J_d[6] = -tau_max_2;

    for(unsigned int i=0; i<7; i++)
    {
        if(st.dq[i] > 2.0 && jimp.y.tau_J_d[i] > 0)
        {
            jimp.y.tau_J_d[i] = 0;
        }

        if(st.dq[i] < -2.0 && jimp.y.tau_J_d[i] < 0)
        {
            jimp.y.tau_J_d[i] = 0;
        }
    }

    return {jimp.y.tau_J_d[0], jimp.y.tau_J_d[1], jimp.y.tau_J_d[2], jimp.y.tau_J_d[3], jimp.y.tau_J_d[4], jimp.y.tau_J_d[5], jimp.y.tau_J_d[6]};
}

void Skill_1::set_U(franka::RobotState st)
{
    jimp.u.M = Eigen::Map<Eigen::Matrix<double, 7, 7>>(model.mass(st).data());
    jimp.u.theta = Eigen::Map<Eigen::Matrix<double, 7, 1>>(st.theta.data());
    jimp.u.dtheta = Eigen::Map<Eigen::Matrix<double, 7, 1>>(st.dtheta.data());
    jimp.u.tau_ff.setZero();
}

void Skill_1::spin()
{
    Eigen::Matrix<double,7,1> q = Eigen::Map<Eigen::Matrix<double,7,1>>(last_state.q.data());
    Eigen::Matrix<double,7,1> dq = Eigen::Map<Eigen::Matrix<double,7,1>>(last_state.dq.data());
    double dist = (q - this->q_init).norm();
    double speed = dq.norm();
    if( dist < 0.1 && speed < 0.1 )
    {
        wave(terminated);
    }

}
