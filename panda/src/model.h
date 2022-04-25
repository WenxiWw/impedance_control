#ifndef MODEL_H
#define MODEL_H

#include "franka/model.h"
#include "franka/robot_state.h"
#include "mujoco.h"

class ModelBase;

class Model
{
public:
    Model();
    std::array< double, 49 > mass(const franka::RobotState& robot_state);
    std::array< double, 42 > bodyJacobian (franka::Frame frame, const franka::RobotState &robot_state);
    std::array< double, 42 > zeroJacobian (franka::Frame frame, const franka::RobotState &robot_state);
    std::array< double, 7 > coriolis (const franka::RobotState &robot_state);
    std::array< double, 7 > coriolis (const std::array<double,7> & q, const std::array<double,7> & dq, const std::array<double,9> & I_total, double m_total, const std::array<double,3> & F_x_Ctotal);
    std::array< double, 7 > gravity (const std::array<double,7> & q, double m_total, const std::array<double,3> & F_x_Ctotal);
    std::array< double, 7 > gravity (const franka::RobotState &robot_state);
    std::array< double, 16 > pose (franka::Frame frame, const franka::RobotState &robot_state);

    void set_modelbase(ModelBase *modelbase);
private:
    ModelBase *modelbase = nullptr;
};

class ModelBase
{
public:
    ModelBase();
    virtual ~ModelBase() {}
    virtual std::array< double, 49 > mass(const franka::RobotState& robot_state) = 0;
    virtual std::array< double, 42 > bodyJacobian (franka::Frame frame, const franka::RobotState &robot_state) = 0;
    virtual std::array< double, 42 > zeroJacobian (franka::Frame frame, const franka::RobotState &robot_state) = 0;
    virtual std::array< double, 7 > coriolis (const std::array<double,7> & q, const std::array<double,7> & dq, const std::array<double,9> & I_total, double m_total, const std::array<double,3> & F_x_Ctotal) = 0;
    virtual std::array< double, 7 > coriolis (const franka::RobotState &robot_state) = 0;
    virtual std::array< double, 7 > gravity (const std::array<double,7> & q, double m_total, const std::array<double,3> & F_x_Ctotal) = 0;
    virtual std::array< double, 7 > gravity (const franka::RobotState &robot_state) = 0;
    virtual std::array< double, 16 > pose (franka::Frame frame, const franka::RobotState &robot_state) = 0;

};

class Model_real : public ModelBase
{
private:
    franka::Model *model;
public:
    Model_real(franka::Model *model);
    std::array< double, 49 > mass(const franka::RobotState& robot_state);
    std::array< double, 42 > bodyJacobian (franka::Frame frame, const franka::RobotState &robot_state);
    std::array< double, 42 > zeroJacobian (franka::Frame frame, const franka::RobotState &robot_state);
    std::array< double, 7 > coriolis (const std::array<double,7> & q, const std::array<double,7> & dq, const std::array<double,9> & I_total, double m_total, const std::array<double,3> & F_x_Ctotal);
    std::array< double, 7 > coriolis (const franka::RobotState &robot_state);
    std::array< double, 7 > gravity (const std::array<double,7> & q, double m_total, const std::array<double,3> & F_x_Ctotal);
    std::array< double, 7 > gravity (const franka::RobotState &robot_state);
    std::array< double, 16 > pose (franka::Frame frame, const franka::RobotState &robot_state);

    virtual ~Model_real() {}
};

class Model_sim : public ModelBase
{
private:
    mjModel *m;
    mjData *d;
    int frame2mujoco(franka::Frame frame);

public:
    Model_sim(std::string mujoco_path);
    ~Model_sim();
    std::array< double, 49 > mass(const franka::RobotState& robot_state);
    std::array< double, 49 > mass_data;
    std::array< double, 42 > bodyJacobian (franka::Frame frame, const franka::RobotState &robot_state);
    std::array< double, 42 > bJ_data;
    std::array< double, 42 > zeroJacobian (franka::Frame frame, const franka::RobotState &robot_state);
    std::array< double, 42 > zJ_data;
    std::array< double, 7 > coriolis (const franka::RobotState &robot_state);
    std::array< double, 7 > coriolis (const std::array<double,7> & q, const std::array<double,7> & dq, const std::array<double,9> & I_total, double m_total, const std::array<double,3> & F_x_Ctotal);
    std::array< double, 7 > gravity (const franka::RobotState &robot_state);
    std::array< double, 7 > gravity (const std::array<double,7> & q, double m_total, const std::array<double,3> & F_x_Ctotal);
    std::array< double, 16 > pose (franka::Frame frame, const franka::RobotState &robot_state);

};



#endif // MODELBASE_H
