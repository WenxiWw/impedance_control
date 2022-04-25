#include "flags/flag.h"
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <array>
#include <iostream>
#include "skills/cart_pd/skill_cart_pd.h"
#include <Eigen/Dense>

Skill_Cart_PD::Skill_Cart_PD()
{
    state_publisher = nh.advertise<panda_msgs_srvs::robot_state_cart>("robot_state_cart", 10);
}

void Skill_Cart_PD::initialize(franka::RobotState st)
{
    translational_stiffness = 900;
    rotational_stiffness = 100;
    translational_damping = 60;
    rotational_damping = 20;

    initial_pose = Eigen::Map<Eigen::Matrix<double,4,4>>(st.O_T_EE.data());
    t_0 =ros::Time::now().toSec();

    q_0 = Eigen::Map<Eigen::Matrix<double,7,1>>(st.q.data());
    stiffness_torque << 900, 900, 900, 900, 100, 100, 100;

    time=0;
}

franka::Torques Skill_Cart_PD::control(franka::RobotState st)
{
    /*
    The robot is already in gravity compensation mode
    */

	
	

	//double Kp_r,Kp_t,Kd_r,Kp_d;
        Eigen::Matrix<double,6,1> Kp_vec,Kd_vec,dx;
        Eigen::Matrix<double,6,6> Kp,Kd,x_des;
        Eigen::Matrix<double,7,1> C;
        Eigen::Matrix<double,6,7> J;



	//motion generator -> transformation matrix 4x4
	Eigen::Matrix<double,4,4> T_des=mo_gen();

	//Kp Kd matrix
	//Kp=diag 900,900,900,100,100,100;
	//Kd=diag 20,20,20,60,60,60;
	Kp_vec << translational_stiffness,translational_stiffness,translational_stiffness,rotational_stiffness,rotational_stiffness,rotational_stiffness;
	Kd_vec << translational_damping,translational_damping,translational_damping,rotational_damping,rotational_damping,rotational_damping;
	Kp = Kp_vec.asDiagonal();
	Kd = Kd_vec.asDiagonal();


	//get Coriolis matrix, Jacobian
	C=Eigen::Map<Eigen::Matrix<double,7,1>>(model.coriolis(st.q,st.dq,st.I_total,st.m_total,st.F_x_Ctotal).data());
	
	
	std::array<double,42>  J_array = model.zeroJacobian(franka::Frame::kEndEffector,st);
	J = Eigen::Map<Eigen::Matrix<double,6,7>>(J_array.data(),6,7);
	
	
	
	
	//transformation matrix -> position & quaterniond
	Eigen::Affine3d trans(Eigen::Matrix4d::Map(st.O_T_EE.data()));
	Eigen::Vector3d pos(trans.translation());
	Eigen::Quaterniond rot(trans.linear());
	

	Eigen::Affine3d trans_d(T_des);
	Eigen::Vector3d pos_d(trans_d.translation());
	Eigen::Quaterniond rot_d(trans_d.linear());
	

	
	//q dq dx=Jacobian * dq
	Eigen::Matrix<double,7,1> q,dq;
	q=Eigen::Map<Eigen::Matrix<double,7,1>>(st.q.data());
	dq=Eigen::Map<Eigen::Matrix<double,7,1>>(st.dq.data());
	dx=J*dq;
	
	//position error
	Eigen::Matrix<double, 6, 1> error;
	error.head(3) << pos_d-pos;

	// difference in  quaternion
	Eigen::Quaterniond error_quaternion(rot_d * rot.inverse());
	//transform to angle xis
	Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
	
	//direction error = angle * axis
	error.tail(3) << error_quaternion_angle_axis.angle() * error_quaternion_angle_axis.axis();
 

	
	

	//tau_task: tau=Jacobian_transpose * F

	Eigen::VectorXd tau_task(7), tau(7),tau_null(7);
	tau_task << J.transpose() *( Kp * error-  Kd * dx );
	


    // ------------------ Null-space CONTROLLER ------------------

    
	//Jacobian pseudo inverse
	Eigen::Matrix<double,6,6> J_temp=J*J.transpose();
        Eigen::Matrix<double,7,6> J_pinv=J.transpose()*J_temp.inverse();

	//calculate tau_null space
	Eigen::Matrix<double,7,7> stiffness_torque_mtr = stiffness_torque.asDiagonal();
	Eigen::Matrix<double,7,1> torque_null = stiffness_torque_mtr * (q_0-q);
	tau_null<< (Eigen::MatrixXd::Identity(7,7) - J.transpose() * J_pinv.transpose()) * torque_null;


	//compute tau
	tau=tau_task+tau_null+C;

	std::array<double,7> tau_array{};
        Eigen::VectorXd::Map(&tau_array[0], 7) = tau;




	
	

	//ROS publisher
	boost::array<double, 3> position_array{};
        Eigen::VectorXd::Map(&position_array[0], 3) = pos;
        msg.position_linear=position_array;

        boost::array<double, 3> position_d_array{};
        Eigen::VectorXd::Map(&position_d_array[0], 3) = pos_d;
        msg.position_linear_d=position_d_array;

        boost::array<double, 6> error_array{};
        Eigen::VectorXd::Map(&error_array[0], 6) = error;
        msg.error_linear_x=error_array;

        boost::array<double, 7> tau_task_array{};
        Eigen::VectorXd::Map(&tau_task_array[0], 7) = tau_task;
        msg.tau_prim=tau_task_array;

	boost::array<double, 7> tau_null_array{};
        Eigen::VectorXd::Map(&tau_null_array[0], 7) = tau_null;
        msg.tau_sec=tau_null_array;


        double t=ros::Time::now().toSec();
        msg.time=t;


	state_publisher.publish(msg);

	return tau_array;
    
}

Eigen::Matrix<double,4,4> Skill_Cart_PD::mo_gen()
{
    Eigen::Matrix<double,4,4> pose_d;

    time += 0.001;

    constexpr double kRadius = 0.1;
    double angle = 0.25*time*2*M_PI;
    double delta_y = kRadius * std::sin(angle);
    double delta_z = kRadius * (std::cos(angle) - 1);
    pose_d= initial_pose;
    pose_d(1,3) += delta_y;
    pose_d(2,3) += delta_z;

    return pose_d;
}

void Skill_Cart_PD::spin()
{

}
