#include "task_cart_pd.h"

Skill Task_Cart_PD::statemachine(Skill skill, Flag flag)
{
    if(flag == init)
    {
        ROS_INFO("Go to initial position");
        return home_skill;
    }
    if(skill == home_skill)
    {
        if(flag == terminated)
        {
            ROS_INFO("Terminated. Switching to Cartesian Impedance Skill");
            return cartesian_pd_skill;
        }
        else if(flag == aborted)
        {
            ROS_INFO("Aborted! Switching to Cartesian Impedance Skill");
            return cartesian_pd_skill;
        }
    }
    else if (skill == cartesian_pd_skill)
    {
        if(flag == terminated)
        {
            ROS_INFO("Terminated. Switching to Home Skill");
            return home_skill;
        }
        else if(flag == aborted)
        {
            ROS_INFO("Aborted! Switching to Home Skill");
            return home_skill;
        }
    }
    ROS_INFO("Error in the state machine!");
    return skill_0;
}
