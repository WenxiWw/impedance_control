#ifndef ROBOT_REAL_H
#define ROBOT_REAL_H

#include "robotbase.h"
#include <franka/robot.h>
#include <franka/model.h>
#include <thread>


class Robot_real : public RobotBase
{
public:
  Robot_real(const std::string ip_address, TaskBase *task);
  ~Robot_real();
//  State read_once();
private:
  //franka::Robot *robot;
  //static franka::Robot *robot_static;
  //franka::Model *model;
  //static franka::Model *model_static;
  //static Controller *ctrl_static;
  std::thread *control_thread;
  Model_real *model_real;
  Gripper_real *gripper_real;

  //static franka::Torques control_callback(const franka::RobotState& robot_state, franka::Duration period);

  void control_thread_function(const std::string ip_address);

  void setDefaultBehavior(franka::Robot& robot);

  //void (franka::Robot::*control_thread_function)(std::function<franka::Torques(const franka::RobotState&, franka::Duration)>, bool, double);
};

#endif // ROBOT_REAL_H
