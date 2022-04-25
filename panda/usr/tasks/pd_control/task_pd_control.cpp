#include "tasks/pd_control/task_pd_control.h"

Skill Task_PD_Control::statemachine(Skill skill, Flag flag)
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
            ROS_INFO("Terminated. Switching to PD Control Skill");
            return pd_control_skill;
        }
        else if(flag == aborted)
        {
            ROS_INFO("Aborted! Switching to PD Control Skill");
            return pd_control_skill;
        }
    }
    else if (skill == pd_control_skill)
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