#ifndef TASK_PD_CONTROL_H
#define TASK_PD_CONTROL_H

#include "taskbase.h"
#include "skills/pd_control/skill_pd_control.h"

class Task_PD_Control : public TaskBase
{
    //list all Skills that are used in the state machine and define their Types ("new Skill_Type")
    Skill pd_control_skill = new Skill_PD_Control;
    Skill home_skill = new Skill_1;
    Skill skill_0 = new Skill_0;

    Skill statemachine(Skill skill, Flag flag);
};

#endif // TASK_TEST_H
