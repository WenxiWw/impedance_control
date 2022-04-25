#ifndef TASK_TEST_H
#define TASK_TEST_H

#include "taskbase.h"
#include "skills/test/skill_test.h"

class Task_Test : public TaskBase
{
    //list all Skills that are used in the state machine and define their Types ("new Skill_Type")
    Skill testskill1 = new Skill_Test(1.0);
    Skill testskill2 = new Skill_Test(1.5);
    Skill skill_0 = new Skill_0;

    Skill statemachine(Skill skill, Flag flag);
};

#endif // TASK_TEST_H
