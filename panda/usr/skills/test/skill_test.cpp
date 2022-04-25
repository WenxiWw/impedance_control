#include "tasks/test/task_test.h"
#include "flags/flag.h"

Skill_Test::Skill_Test(double q_d)
{
    this->q_d << 0, 0, 0, -1, 0, q_d, 0;

    subscriber = nh.subscribe("abort", 1, &Skill_Test::callback, this);
}

void Skill_Test::initialize(franka::RobotState st)
{
  cji.p.D_theta.setConstant(1);
  cji.p.K_theta << 1000, 1000, 1000, 1000, 200, 200, 200;
  cji.p.enable_ffwd_acc << 0;
  cji.p.enable_ffwd_vel << 0;

  set_U(st);

  cji.initialize();

//  if(q_d[3] > 0)
//  {
//    gripper.homing();
//  }
//  else
//  {
//    gripper.move(0, 0.1);
//  }
}

franka::Torques Skill_Test::control(franka::RobotState st)
{
  set_U(st);

  cji.step();

  const double tau_max_1 = 2;
  const double tau_max_2 = 2;

  if(cji.y.tau_J_d[0] > tau_max_1) cji.y.tau_J_d[0] = tau_max_1;
  if(cji.y.tau_J_d[1] > tau_max_1) cji.y.tau_J_d[1] = tau_max_1;
  if(cji.y.tau_J_d[2] > tau_max_1) cji.y.tau_J_d[2] = tau_max_1;
  if(cji.y.tau_J_d[3] > tau_max_1) cji.y.tau_J_d[3] = tau_max_1;
  if(cji.y.tau_J_d[4] > tau_max_2) cji.y.tau_J_d[4] = tau_max_2;
  if(cji.y.tau_J_d[5] > tau_max_2) cji.y.tau_J_d[5] = tau_max_2;
  if(cji.y.tau_J_d[6] > tau_max_2) cji.y.tau_J_d[6] = tau_max_2;

  if(cji.y.tau_J_d[0] < -tau_max_1) cji.y.tau_J_d[0] = -tau_max_1;
  if(cji.y.tau_J_d[1] < -tau_max_1) cji.y.tau_J_d[1] = -tau_max_1;
  if(cji.y.tau_J_d[2] < -tau_max_1) cji.y.tau_J_d[2] = -tau_max_1;
  if(cji.y.tau_J_d[3] < -tau_max_1) cji.y.tau_J_d[3] = -tau_max_1;
  if(cji.y.tau_J_d[4] < -tau_max_2) cji.y.tau_J_d[4] = -tau_max_2;
  if(cji.y.tau_J_d[5] < -tau_max_2) cji.y.tau_J_d[5] = -tau_max_2;
  if(cji.y.tau_J_d[6] < -tau_max_2) cji.y.tau_J_d[6] = -tau_max_2;

  for(unsigned int i=0; i<7; i++)
  {
      if(st.dq[i] > 2.0 && cji.y.tau_J_d[i] > 0)
      {
          cji.y.tau_J_d[i] = 0;
      }

      if(st.dq[i] < -2.0 && cji.y.tau_J_d[i] < 0)
      {
          cji.y.tau_J_d[i] = 0;
      }
  }

  model.zeroJacobian(franka::Frame::kEndEffector, st);

//  std::cout << "tau_J_d:" << std::endl;
//  std::cout << out_Y.tau_J_d << std::endl;
  return{cji.y.tau_J_d[0], cji.y.tau_J_d[1], cji.y.tau_J_d[2], cji.y.tau_J_d[3], cji.y.tau_J_d[4], cji.y.tau_J_d[5], cji.y.tau_J_d[6]};

//  return{0, 0, 0, 0, 0, 0, 0};
}

void Skill_Test::spin()
{
    Eigen::Matrix<double,7,1> q = Eigen::Map<Eigen::Matrix<double,7,1>>(last_state.q.data());
    Eigen::Matrix<double,7,1> dq = Eigen::Map<Eigen::Matrix<double,7,1>>(last_state.dq.data());
    double dist = (q - q_d).norm();
    double speed = dq.norm();
    if( dist < 0.1 && speed < 0.1 )
    {
        wave(terminated);
    }

}

void Skill_Test::set_U(franka::RobotState st)
{
    cji.u.M = Eigen::Map<Eigen::Matrix<double,7,7>>(model.mass(st).data());

    cji.u.theta = Eigen::Map<Eigen::Matrix<double,7,1>>(st.q.data());
    cji.u.dtheta = Eigen::Map<Eigen::Matrix<double,7,1>>(st.dq.data());
    cji.u.tau_ff.setZero();
    cji.u.theta_d = q_d;
    cji.u.dtheta_d.setZero();
    cji.u.ddtheta_d.setZero();
}

void Skill_Test::callback(std_msgs::Header abort)
{
    wave(aborted);
}
