#ifndef SKILL_CART_PD_H
#define SKILL_CART_PD_H

#include "skillbase.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "panda_msgs_srvs/robot_state_cart.h"

#include <franka/model.h>

class Skill_Cart_PD: public SkillBase
{
  private:
    ros::NodeHandle nh;
    ros::Publisher state_publisher;
    panda_msgs_srvs::robot_state_cart msg;

    void initialize(franka::RobotState st);
    franka::Torques control(franka::RobotState st);
    Eigen::Matrix<double,4,4> mo_gen();

    // Compliance parameters
    double translational_stiffness;
    double rotational_stiffness;
    double translational_damping;
    double rotational_damping;

    Eigen::Matrix<double,4,4> initial_pose;
    double t_0;
    Eigen::Matrix<double,7,1> q_0;
    Eigen::Matrix<double,7,1> stiffness_torque;
    double time;



    void spin();

    void callback (std_msgs::Header abort);

public:
    Skill_Cart_PD();

};

#endif // SKILL_CART_PD_H
