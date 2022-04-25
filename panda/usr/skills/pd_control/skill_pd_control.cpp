#include "flags/flag.h"
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <array>
#include <iostream>
#include <skills/pd_control/skill_pd_control.h>
#include <vector>
#include <Eigen/Dense>
#include "panda_msgs_srvs/robot_state_pd.h"

Skill_PD_Control::Skill_PD_Control()
{
    state_publisher = nh.advertise<panda_msgs_srvs::robot_state_pd>("robot_state_pd", 10);
}

void Skill_PD_Control::initialize(franka::RobotState st)
{
    /*
    Init function

    Initialize gains K_p, K_d, starting time t_0 and initial position
    */

    K_p << 900, 900, 900, 900, 100, 100, 100;
    K_d << 60, 60, 60, 60, 20, 20, 20;

    initial_position = Eigen::Map<Eigen::Matrix<double,7,1>>(st.q.data());
    t_0 = ros::Time::now().toSec();

    
}


franka::Torques Skill_PD_Control::control(franka::RobotState st)
{
    /*
    Control function. This function is called 
    The robot is already in gravity compensation mode to avoid unpredictable outcome.

    Inputs:
    ------
        - st (franka::RobotState): Current state of the robot. Including
    Return:
        - tau_d_array (std::array<double,7>): Control torque
    */
    


	
	
	//motion generator - desired position, to struct des
	des desi=mo_gen();
	
	
	Eigen::Matrix<double,7,1> q,dq,q_d,dq_d,e,de,tor,C;
	Eigen::Matrix<double,7,7> kp,kd,mass;


	//state q dq
	q=Eigen::Map<Eigen::Matrix<double,7,1>>(st.q.data());
	dq=Eigen::Map<Eigen::Matrix<double,7,1>>(st.dq.data());



	//desired joint position dq_d
	std::array<double,7> dq_d_array{};
	Eigen::VectorXd::Map(&dq_d_array[0], 7) = desi.dq_d;
	//coriolis matrix
	C=Eigen::Map<Eigen::Matrix<double,7,1>>(model.coriolis(st.q,dq_d_array,st.I_total,st.m_total,st.F_x_Ctotal).data());

	//Mass matrix
	//mass=Eigen::Map<Eigen::Matrix<double,7,7>>(model.mass(st).data());


	//error q and dq from desired value
	e=desi.q_d-q;
	de=desi.dq_d-dq;

	//coriolis=Eigen::Map<Eigen::Matrix<double,7,7>>(Model.coriolis.data());


	//Kp Kd matrix
	kp = K_p.asDiagonal();
       kd = K_d.asDiagonal();
	
       //output torque
	tor=kp*e+kd*de+C;

	std::array<double,7> tau_array{};
	Eigen::VectorXd::Map(&tau_array[0], 7) = tor;


	
	//ROS publisher
	
	boost::array<double, 7> q_array{};
	Eigen::VectorXd::Map(&q_array[0], 7) = q;
	msg.q=q_array;

	boost::array<double, 7> dq_array{};
	Eigen::VectorXd::Map(&dq_array[0], 7) = dq;
	msg.dq=dq_array;
	
	boost::array<double, 7> q_d_array{};
	Eigen::VectorXd::Map(&q_d_array[0], 7) = desi.q_d;
        msg.q_d=q_d_array;

	boost::array<double, 7> err_q_array{};
        Eigen::VectorXd::Map(&err_q_array[0], 7) = e;
        msg.err_q=err_q_array;

	double t=ros::Time::now().toSec();
	msg.time=t;

        
	state_publisher.publish(msg);

	


	return tau_array;
}

Skill_PD_Control::des Skill_PD_Control::mo_gen()
{   
    double t = ros::Time::now().toSec() - t_0;

    Skill_PD_Control::des desired;
    
    Eigen::Matrix<double,7,1> q_desired;
    double delta_angle = M_PI / 4 * (std::sin(M_PI / 8 * t));
    desired.q_d << initial_position[0], initial_position[1],
            initial_position[2], initial_position[3],
            initial_position[4]+ delta_angle, initial_position[5],
            initial_position[6];

    desired.dq_d << 0, 0, 0, 0, M_PI / 4 * std::cos(M_PI / 8 * t) * M_PI / 8, 0, 0;
    desired.ddq_d << 0, 0, 0, 0, -M_PI /4 * std::sin(M_PI / 8 * t) * M_PI / 8 * M_PI / 8, 0, 0;

    return desired;
}


void Skill_PD_Control::spin()
{

}
