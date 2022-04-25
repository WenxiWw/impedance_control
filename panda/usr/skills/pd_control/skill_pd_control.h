#ifndef SKILL_PD_CONTROL_H
#define SKILL_PD_CONTROL_H

#include "skillbase.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "panda_msgs_srvs/robot_state_pd.h"

#include <franka/model.h>

class Skill_PD_Control : public SkillBase
{
  private:
    ros::NodeHandle nh;
    ros::Publisher state_publisher;
    panda_msgs_srvs::robot_state_pd msg;

    struct des
    {
        Eigen::Matrix<double,7,1> q_d;
        Eigen::Matrix<double,7,1> dq_d;
        Eigen::Matrix<double,7,1> ddq_d;
    };

    void initialize(franka::RobotState st);
    franka::Torques control(franka::RobotState st);
    des mo_gen();

    // Set gains for the joint impedance control.
    Eigen::Matrix<double,7,1> K_p;
    Eigen::Matrix<double,7,1> K_d;
    Eigen::Matrix<double,7,1> initial_position;
    double t_0;
        
    void spin();
    void set_U(franka::RobotState st);

    void callback (std_msgs::Header abort);
public:
    Skill_PD_Control();
};

#endif
