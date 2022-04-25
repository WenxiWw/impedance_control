#ifndef SKILL_TEST_H
#define SKILL_TEST_H

#include "skillbase.h"
#include "controllers/cntr_joint_imp/cntr_joint_imp/cntr_joint_imp_wrapper.hpp"
#include "std_msgs/Header.h"

class Skill_Test : public SkillBase
{
  private:
    ros::NodeHandle nh;
    ros::Subscriber subscriber;

    void initialize(franka::RobotState st);
    franka::Torques control(franka::RobotState st);

    cntr_joint_imp::cntr_joint_imp cji;

    Eigen::Matrix<double,7,1> q_d;

    void spin();
    void set_U(franka::RobotState st);

    void callback (std_msgs::Header abort);

public:
    Skill_Test(double q_d=0);

};

#endif // SKILL_TEST_H
