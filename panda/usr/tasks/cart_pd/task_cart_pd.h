#ifndef TASK_CART_PD_H
#define TASK_CART_PD_H

#include "taskbase.h"
#include "skills/cart_pd/skill_cart_pd.h"

class Task_Cart_PD : public TaskBase
{
    //list all Skills that are used in the state machine and define their Types ("new Skill_Type")
    Skill cartesian_pd_skill = new Skill_Cart_PD;
    Skill home_skill = new Skill_1;
    Skill skill_0 = new Skill_0;

    Skill statemachine(Skill skill, Flag flag);
};

#endif // TASK_CART_PD_H
