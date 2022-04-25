#include "model.h"
#include "ros/ros.h"

#include "eigen3/Eigen/Core"

Model::Model()
{

}

std::array< double, 49 > Model::mass(const franka::RobotState &robot_state)
{
    return modelbase->mass(robot_state);
}

std::array< double, 42 > Model::bodyJacobian (franka::Frame frame, const franka::RobotState &robot_state)
{
    return modelbase->bodyJacobian(frame, robot_state);
}

std::array< double, 42 > Model::zeroJacobian (franka::Frame frame, const franka::RobotState &robot_state)
{
    return modelbase->zeroJacobian(frame, robot_state);
}

std::array< double, 7 > Model::coriolis(const franka::RobotState &robot_state)
{
    return modelbase->coriolis(robot_state);
}

std::array< double, 7 > Model::coriolis (const std::array<double,7> & q, const std::array<double,7> & dq, const std::array<double,9> & I_total, double m_total, const std::array<double,3> & F_x_Ctotal)
{
    return modelbase->coriolis(q, dq, I_total, m_total, F_x_Ctotal);
}

std::array< double, 7 > Model::gravity(const franka::RobotState &robot_state)
{
    return modelbase->gravity(robot_state);
}

std::array< double, 7 > Model::gravity(const std::array<double,7> & q, double m_total, const std::array<double,3> & F_x_Ctotal)
{
    return modelbase->gravity(q, m_total, F_x_Ctotal);
}

std::array< double, 16 > Model::pose (franka::Frame frame, const franka::RobotState &robot_state)
{
    return modelbase->pose(frame, robot_state);
}

void Model::set_modelbase(ModelBase *modelbase)
{
    this->modelbase = modelbase;
}


//--------------------------------------------



ModelBase::ModelBase()
{

}

Model_real::Model_real(franka::Model *model)
{
    this->model = model;
}

std::array< double, 49 > Model_real::mass(const franka::RobotState& robot_state)
{
    return model->mass(robot_state);
}

std::array< double, 42 > Model_real::bodyJacobian (franka::Frame frame, const franka::RobotState &robot_state)
{
    return model->bodyJacobian(frame, robot_state);
}

std::array< double, 42 > Model_real::zeroJacobian (franka::Frame frame, const franka::RobotState &robot_state)
{
    return model->zeroJacobian(frame, robot_state);
}

std::array< double, 7 > Model_real::coriolis(const franka::RobotState &robot_state)
{
    return model->coriolis(robot_state);
}

std::array< double, 7 > Model_real::coriolis(const std::array<double,7> & q, const std::array<double,7> & dq, const std::array<double,9> & I_total, double m_total, const std::array<double,3> & F_x_Ctotal)
{
    return model->coriolis(q, dq, I_total, m_total, F_x_Ctotal);
}

std::array< double, 7 > Model_real::gravity(const franka::RobotState &robot_state)
{
    return model->gravity(robot_state);
}

std::array< double, 7 > Model_real::gravity(const std::array<double,7> & q, double m_total, const std::array<double,3> & F_x_Ctotal)
{
    return model->gravity(q, m_total, F_x_Ctotal);
}

std::array< double, 16 > Model_real::pose (franka::Frame frame, const franka::RobotState &robot_state)
{
    return model->pose(frame, robot_state);
}

//----------------------------------------


Model_sim::Model_sim(std::string mujoco_path)
{
    std::string file_key = std::string(mujoco_path) + std::string("/mujoco_equip/key/mjkey.txt");
    std::string file_model = std::string(mujoco_path) + std::string("/mujoco_equip/model/panda.xml");

    mj_activate(file_key.c_str());

    ROS_INFO("MuJoCo activated for model");

    char error[500] = "";
    m = mj_loadXML(file_model.c_str(), nullptr, error, 500);
    d = mj_makeData(m);
    if(!m)
    {
        ROS_ERROR("%s\n", error);
    }
}

Model_sim::~Model_sim()
{
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();
}

int Model_sim::frame2mujoco(franka::Frame frame)
{
    //int ee = 10; // body number of endeffector

    int body=0;
    switch(frame)
    {
        case franka::Frame::kJoint1: body=2; break;
        case franka::Frame::kJoint2: body=3; break;
        case franka::Frame::kJoint3: body=4; break;
        case franka::Frame::kJoint4: body=5; break;
        case franka::Frame::kJoint5: body=6; break;
        case franka::Frame::kJoint6: body=7; break;
        case franka::Frame::kJoint7: body=8; break;
        case franka::Frame::kFlange: body=9; break;
        case franka::Frame::kEndEffector: body=10; break;
        case franka::Frame::kStiffness: body=10; ROS_WARN("not implemented for stiffness frame! using Endeffector instead"); break;
    }
    return body;

}

std::array< double, 49 > Model_sim::mass(const franka::RobotState& robot_state)
{
    for (unsigned long i=0; i<7; i++)
    {
        d->qpos[i] = robot_state.q[i];
    }
    mj_forward(m, d);

    const int nv = 9;//m->nv;
    mjtNum M[nv*nv];
    mj_fullM(m, M, d->qM);

    Eigen::Matrix<double,7,7> M_mat = Eigen::Map<Eigen::Matrix<double,nv,nv>>(M, nv, nv).block(0,0,7,7);
    //Eigen::Matrix<double,7,7> M_old = Eigen::Map<Eigen::Matrix<double,7,7>>(mass_data.data());

    std::array< double,49 > result;

    for(unsigned long i=0; i<49; i++)
    {
        result[i] = M_mat.data()[i];
    }

    return result;
}

std::array< double, 42 > Model_sim::bodyJacobian (franka::Frame frame, const franka::RobotState &robot_state) //tested
{
    Eigen::Matrix<double,6,7> zJ = Eigen::Map<Eigen::Matrix<double,6,7>>(zeroJacobian(frame, robot_state).data());
    Eigen::Matrix<double,4,4> p = Eigen::Map<Eigen::Matrix<double,4,4>>(pose(frame,robot_state).data());
    Eigen::Matrix<double,3,3> rot = p.block(0,0,3,3);

    Eigen::Matrix<double,3,7> bJp;
    Eigen::Matrix<double,3,7> bJr;

    bJp = rot.transpose()*zJ.block(0,0,3,7);
    bJr = rot.transpose()*zJ.block(3,0,3,7);

    Eigen::Matrix<double,6,7> bJ;

    bJ << bJp, bJr;

    std::array<double, 42> result;

    for(unsigned long i=0; i<42; i++)
    {
        result[i] = bJ.data()[i];
    }

    //return bJ_data;
    return result;
}

std::array< double, 42 > Model_sim::zeroJacobian (franka::Frame frame, const franka::RobotState &robot_state) //tested function
{

    int body=frame2mujoco(frame);

    for (unsigned long i=0; i<7; i++)
    {
        d->qpos[i] = robot_state.q[i];
    }
    mj_forward(m, d);

    const int nv = 9;//m->nv;

    mjtNum jacp[3*nv];
    mjtNum jacr[3*nv];
    mj_jacBody(m, d, jacp, jacr, body);

    Eigen::Matrix<double,9,3> jacp_matrix(jacp);
    Eigen::Matrix<double,9,3> jacr_matrix(jacr);
    Eigen::Matrix<double,7,3> jacp_matrix_cut = jacp_matrix.block(0,0,7,3);
    Eigen::Matrix<double,7,3> jacr_matrix_cut = jacr_matrix.block(0,0,7,3);

    Eigen::Matrix<double,6,7> zJ_new;
    zJ_new << jacp_matrix_cut.transpose(), jacr_matrix_cut.transpose();

    //Eigen::Matrix<double,6,7> zJ_old = Eigen::Map<Eigen::Matrix<double,6,7>>(zJ_data.data());

    std::array< double,42 > result;

    for(unsigned long i=0; i<42; i++)
    {
        result[i] = zJ_new.data()[i];
    }

    return result;
}

std::array< double, 7 > Model_sim::coriolis(const franka::RobotState &robot_state) // tested: reasonable performance
{
    return coriolis(robot_state.q, robot_state.dq, robot_state.I_total, robot_state.m_total, robot_state.F_x_Ctotal);
}

std::array< double, 7 > Model_sim::coriolis (const std::array<double,7> & q, const std::array<double,7> & dq, const std::array<double,9> & I_total, double m_total, const std::array<double,3> & F_x_Ctotal)
{
    for (unsigned long i=0; i<7; i++)
    {
        d->qpos[i] = q[i];
        d->qvel[i] = dq[i];
    }

    mj_forward(m, d);

    Eigen::Matrix<double,7,1> bias(d->qfrc_bias);

    Eigen::Matrix<double,7,1> grav(gravity(q, m_total, F_x_Ctotal).data());

    Eigen::Matrix<double,7,1> coriolis = bias-grav;

    return {coriolis[0], coriolis[1], coriolis[2], coriolis[3], coriolis[4], coriolis[5], coriolis[6]};
}

std::array< double, 7 > Model_sim::gravity(const std::array<double,7> & q, double m_total, const std::array<double,3> & F_x_Ctotal)
{
    for (unsigned long i=0; i<7; i++)
    {
        d->qpos[i] = q[i];
    }

    mj_forward(m, d);

    const int nv = 9;


    //gravity
    mjtNum gravVec[nv];
    for(int i=0; i<nv; i++)
    {
        gravVec[i] = 0; //gravity Vector
    }

    for(int i=2; i<=10; i++) {
        mjtNum jacp[3*nv];
        mjtNum jacr[3*nv];
        mj_jacBodyCom(m, d, jacp, jacr, i); //calculate Jacobian
        mjtNum g[3];
        for(int d=0; d<3; d++)
        {
            g[d] = m->opt.gravity[d] * m->body_mass[i]; //calculate gravity force: G=g*m
        }
        mjtNum tmp[nv];
        mju_mulMatTVec(tmp, jacp, g, 3, nv);
        for(int j=0; j<nv; j++) {
            gravVec[j] -= tmp[j]; //add to gravity Vector
        }
    }

    std::array<double,7> result;
    for (unsigned long i=0; i<7; i++)
    {
        result[i] = gravVec[i];
    }
    return result;
}

std::array< double, 7 > Model_sim::gravity(const franka::RobotState &robot_state) // tested: reasonable performance
{
    return gravity(robot_state.q, robot_state.m_total, robot_state.F_x_Ctotal);
}

std::array< double, 16 > Model_sim::pose (franka::Frame frame, const franka::RobotState &robot_state) //tested: small deviation
{
    int body = frame2mujoco(frame);

    for (unsigned long i=0; i<7; i++)
    {
        d->qpos[i] = robot_state.q[i];
    }

    mj_forward(m, d);

    std::array< double, 16 > result = {d->xmat[body*9],   d->xmat[body*9+3], d->xmat[body*9+6], 0,
                                       d->xmat[body*9+1], d->xmat[body*9+4], d->xmat[body*9+7], 0,
                                       d->xmat[body*9+2], d->xmat[body*9+5], d->xmat[body*9+8], 0,
                                       d->xpos[body*3],   d->xpos[body*3+1], d->xpos[body*3+2], 1}; //transposed for mujoco compared to franka

    return result;
}
