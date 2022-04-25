#include "gripperbase.h"
#include "ros/ros.h"

Gripper::Gripper()
{

}

void Gripper::set_gripperbase(GripperBase *gripperbase)
{
    this->gripperbase = gripperbase;
}

bool Gripper::homing()
{
    return gripperbase->homing();
}

bool Gripper::grasp(double width, double speed, double force, double epsilon_inner, double epsilon_outer)
{
    return gripperbase->grasp(width, speed, force, epsilon_inner, epsilon_outer);
}

bool Gripper::move(double width, double speed)
{
    return gripperbase->move(width, speed);
}

bool Gripper::stop()
{
    return gripperbase->stop();
}


GripperBase::GripperBase()
{

}


Gripper_real::Gripper_real(franka::Gripper *gripper)
{
    this->franka_gripper = gripper;
}

bool Gripper_real::homing()
{
    return franka_gripper->homing();
}

bool Gripper_real::grasp(double width, double speed, double force, double epsilon_inner, double epsilon_outer)
{
    return franka_gripper->grasp(width, speed, force, epsilon_inner, epsilon_outer);
}

bool Gripper_real::move(double width, double speed)
{
    return franka_gripper->move(width, speed);
}

bool Gripper_real::stop()
{
    return franka_gripper->stop();
}


bool Gripper_sim::homing()
{
    gripper_width_d = GRIPPER_WIDTH_MAX;
    ROS_WARN("Homing of simulation of gripper is not implemented yet!");
    return true;
}

bool Gripper_sim::grasp(double width, double speed, double force, double epsilon_inner, double epsilon_outer)
{
    gripper_width_d = 0;
    ROS_WARN("Grasping of simulation of gripper is not realistically implemented yet!");
    return true;
}

bool Gripper_sim::move(double width, double speed)
{
    gripper_width_d = width;
    ROS_WARN("Moving of simulation of gripper is not realistically implemented yet!");
    return true;
}

bool Gripper_sim::stop()
{
    gripper_speed_d = 0;
    return true;
}

Gripper_sim::Torque Gripper_sim::control_gripper(Gripper_sim::State st)
{
    gripper_width_d = gripper_width_d - gripper_speed_d*0.001; //minus because speed references to closing speed!
    if(gripper_width_d > GRIPPER_WIDTH_MAX)
    {
        gripper_width_d = GRIPPER_WIDTH_MAX;
    }
    if(gripper_width_d < 0)
    {
        gripper_width_d = 0;
    }

    Torque cmd;
    cmd.tau_r = GRIPPER_STIFFNESS*(gripper_width_d/2.0 - st.pos_r); // - GRIPPER_DAMPING*st.vel_r;
    cmd.tau_l = GRIPPER_STIFFNESS*(gripper_width_d/2.0 - st.pos_l); // - GRIPPER_DAMPING*st.vel_l;

    return cmd;
}
