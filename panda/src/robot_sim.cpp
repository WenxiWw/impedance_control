#include "robot_sim.h"
#include "eigen3/Eigen/Geometry"

Robot_sim::Robot_sim(std::string mujoco_path, TaskBase *task) : RobotBase (task) , model(mujoco_path)
{
      srv_control = nh.advertiseService("control_sim", &Robot_sim::control, this);
      is_first_dropped = false;
      task->set_modelbase(&model);
      task->set_gripperbase(&gripper);

      for(int i=0; i<7; i++)
      {
          dq_last[i] = 0;
      }
}

bool Robot_sim::control(panda_msgs_srvs::control::Request &req, panda_msgs_srvs::control::Response &res)
{
    if(!is_first_dropped) // drop first call of mujoco due to wrong initialization
    {
        for(unsigned long j=0; j<7; j++)
        {
            res.tau[j] = 0;
        }
        is_first_dropped = true;
        return true;
    }


    franka::RobotState st;

    double tau_gripper[2];

    for(unsigned long j=0; j<7; j++) //convert from boost::array to std::array
    {
        st.q[j] = req.q[j];
        st.dq[j] = 0.2*req.dq[j] + 0.8*dq_last[j];
        dq_last[j] = st.dq[j];
        st.theta[j] = req.q[j];
        st.dtheta[j] = st.dq[j];
    }

    for(unsigned long i=0; i<49; i++) //convert from boost::array to std::array
    {
        model.mass_data[i] = req.M[i];
    }

    //st.M = Eigen::Map<Eigen::Matrix<double,7,7>>( req.M.data() );

    Eigen::Translation3d x_pos_EE(Eigen::Vector3d(req.x_pos_EE.data()));
    Eigen::Matrix3d x_mat_EE(req.x_mat_EE.data());
    Eigen::Affine3d O_T_EE = x_pos_EE * x_mat_EE.transpose();
    //O_T_EE.setIdentity();
    //O_T_EE.translation() = Eigen::Vector3d(req.x_pos_EE.data());
    for(unsigned long i=0; i<16; i++)
    {
        st.O_T_EE[i] = O_T_EE.matrix().data()[i];
    }


    //copying the jacobian into the data field of the model
    Eigen::Matrix<double,7,3> Jp(Eigen::Map<Eigen::Matrix<double,7,3>>(req.bJp.data()));
    Eigen::Matrix<double,7,3> Jr(Eigen::Map<Eigen::Matrix<double,7,3>>(req.bJr.data()));

    //std::cout << "bJp: " << std::endl << bJp.transpose() << std::endl;
    Eigen::Matrix<double,3,7> bJp_trafoed, zJp_trafoed;
    bJp_trafoed = (Jp*x_mat_EE.transpose()).transpose();
    zJp_trafoed = Jp.transpose();
    //std::cout << "bJp_trafoed: " << std::endl << bJp_trafoed << std::endl;
    Eigen::Matrix<double,3,7> bJr_trafoed, zJr_trafoed;
    bJr_trafoed = (Jr*x_mat_EE.transpose()).transpose();
    zJr_trafoed = Jr.transpose();
    //std::cout << "bJr_trafoed: " << std::endl << bJr_trafoed << std::endl;
    Eigen::Matrix<double,6,7> bJ, zJ;
    bJ << bJp_trafoed, bJr_trafoed;
    zJ << zJp_trafoed, zJr_trafoed;
    //st.bJ << bJ; // this is jsut for safety
    for(unsigned long i=0; i<42; i++)
    {
        model.bJ_data[i] = bJ.data()[i];
    }

    for(unsigned long i=0; i<42; i++)
    {
        model.zJ_data[i] = zJ.data()[i];
    }


    Gripper_sim::State st_gripper;
    st_gripper.pos_r = req.q_gripper.at(0);
    st_gripper.pos_l = req.q_gripper.at(1);

    Gripper_sim::Torque torque_gripper = gripper.control_gripper(st_gripper);
    franka::Torques torque = task->control(st); //run the control function of the controller

    for(unsigned long j=0; j<7; j++)
    {
        res.tau[j] = torque.tau_J[j];
    }
    res.tau_gripper[0] = torque_gripper.tau_r;
    res.tau_gripper[1] = torque_gripper.tau_l;

    return true;
}
