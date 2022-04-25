#ifndef GRIPPERBASE_H
#define GRIPPERBASE_H

#include "franka/gripper.h"
#include <string>

#define GRIPPER_WIDTH_MAX 0.1 //maximal gripper width
#define GRIPPER_STIFFNESS 10 //stiffness of the simulated gripper-controller
//#define GRIPPER_DAMPING 2 //damping of the simulated gripper-controller
#define GRIPPER_EPSILON 0.01 //the epsilon of the simulated move function

class GripperBase;

class Gripper
{
public:
    Gripper();
    bool homing();
    bool grasp(double width, double speed, double force, double epsilon_inner=0.005, double epsilon_outer=0.005);
    bool move(double width, double speed);
    bool stop();
    void set_gripperbase(GripperBase *gripperbase);
private:
    GripperBase *gripperbase = nullptr;
};

class GripperBase
{
public:
    GripperBase();
    virtual bool homing() = 0;
    virtual bool grasp(double width, double speed, double force, double epsilon_inner=0.005, double epsilon_outer=0.005) = 0;
    virtual bool move(double width, double speed) = 0;
    virtual bool stop() = 0;
    virtual ~GripperBase() {}
};

class Gripper_real : public GripperBase
{
private:
    franka::Gripper *franka_gripper;
public:
    Gripper_real(franka::Gripper *gripper);
    bool homing();
    bool grasp(double width, double speed, double force, double epsilon_inner=0.005, double epsilon_outer=0.005);
    bool move(double width, double speed);
    bool stop();
};

class Gripper_sim : public GripperBase
{
private:
    double gripper_speed_d = 0;
    double gripper_width_d = 0; //desired width of the gripper

public:
    struct State
    {
        double pos_r;
        double pos_l;
        double vel_r;
        double vel_l;
    };
    struct Torque
    {
        double tau_r;
        double tau_l;
    };

    Torque control_gripper(State state);
    bool homing();
    bool grasp(double width, double speed, double force, double epsilon_inner=0.005, double epsilon_outer=0.005);
    bool move(double width, double speed);
    bool stop();
};

#endif // GRIPPERBASE_H
