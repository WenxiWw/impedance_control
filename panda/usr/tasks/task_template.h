#ifndef TASK_TEMPLATE_H
#define TASK_TEMPLATE_H

#include "taskbase.h"

class Task_Template : public TaskBase
{
    //list all Skills that are used in the state machine and define der Types ("new Skill_Type")
    Skill skill_0 = new Skill_0;

    Skill statemachine(Skill skill, Flag flag);
};

#endif // TASK_TEMPLATE_H
