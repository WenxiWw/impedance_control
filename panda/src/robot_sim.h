#ifndef ROBOT_SIM_H
#define ROBOT_SIM_H

#include "robotbase.h"
#include "panda_msgs_srvs/control.h"
#include <string>


//simulation provides the following entrances of franka::RobotState:
//q, dq, theta (=q), dtheta (=dq), O_T_EE
//and the following entrances of model:
//mass (of the current state), bodyJacobian (of endeffector)

class Robot_sim : public RobotBase
{
private:
    ros::NodeHandle nh;
    ros::ServiceServer srv_control;
    bool control(panda_msgs_srvs::control::Request &req, panda_msgs_srvs::control::Response &res);
    bool is_first_dropped;
    Model_sim model;
    Gripper_sim gripper;

    double dq_last[7];
public:
    Robot_sim(std::string mujoco_path, TaskBase *task);
};

#endif // ROBOT_SIM_H
