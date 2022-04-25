#ifndef SKILLBASE_H
#define SKILLBASE_H

#include <mutex>
#include <vector>

#include "franka/robot_state.h"
#include "ros/ros.h"
#include "eigen3/Eigen/Dense"

#include "flags/flag.h"

#include "model.h"
#include "gripperbase.h"

#include "controllers/cntr_joint_imp/cntr_joint_imp/cntr_joint_imp_wrapper.hpp"

//struct Command
//{
//  double tau[7];
//};

//struct State
//{
//public:
//  franka::RobotState f;
//  //Eigen::Matrix<double,7,7> M; //inertia/Mass matrix
//  //Eigen::Matrix<double,6,7> bJ; //body Jacobian
//};

class SkillBase
{
public:

    private:
        std::mutex mtx;
        bool is_initialised = false;

        virtual franka::Torques control(franka::RobotState state) = 0;

    protected:
        void wave(Flag flag);
        franka::RobotState last_state;

    public:
        bool is_waving = false;
        Flag flag;
        void reset_flag() {is_waving = false;}
        franka::Torques control_wrap(franka::RobotState st);
        virtual ~SkillBase();
        virtual void initialize(franka::RobotState st) = 0;
        void spin_wrap();
        virtual void spin() {}

        Model model;
        void set_modelbase(ModelBase *modelbase);

        Gripper gripper;
        void set_gripperbase(GripperBase *gripperbase);

};

typedef SkillBase* Skill;

class Skill_0 : public SkillBase
{
    void initialize(franka::RobotState st) {}
    franka::Torques control(franka::RobotState st) {return {0, 0, 0, 0, 0, 0, 0};}
    void spin() {wave(init);}

    cntr_joint_imp::cntr_joint_imp jimp;
    void set_U(franka::RobotState st);
};

class Skill_1 : public SkillBase
{
public:
    Skill_1();
    Skill_1(Eigen::Matrix<double, 7, 1> q_init);
private:
    Eigen::Matrix<double, 7, 1> q_init;
    void initialize(franka::RobotState st);
    franka::Torques control(franka::RobotState st);
    void spin();

    cntr_joint_imp::cntr_joint_imp jimp;
    void set_U(franka::RobotState st);
};

class Skill_terminated : public SkillBase{
    void initialize(franka::RobotState st) {}
    franka::Torques control(franka::RobotState st) {return{0, 0, 0, 0, 0, 0, 0};}
    void spin() { }
};

#endif // SKILLBASE_H

