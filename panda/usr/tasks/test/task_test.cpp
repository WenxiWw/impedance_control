#include "task_test.h"

Skill Task_Test::statemachine(Skill skill, Flag flag)
{
    if(flag == init)
    {
        ROS_INFO("Starting with Skill 1");
        return testskill1;
    }
    if(skill == testskill1)
    {
        if(flag == terminated)
        {
            ROS_INFO("Terminated. Switching to Skill 2");
            return testskill2;
        }
        else if(flag == aborted)
        {
            ROS_INFO("Aborted! Switching to Skill 2");
            return testskill2;
        }
    }
    else if (skill == testskill2)
    {
        if(flag == terminated)
        {
            ROS_INFO("Terminated. Switching to Skill 1");
            return testskill1;
        }
        else if(flag == aborted)
        {
            ROS_INFO("Aborted! Switching to Skill 1");
            return testskill1;
        }
    }
    ROS_INFO("error in the state machine!");
    return skill_0;
}
