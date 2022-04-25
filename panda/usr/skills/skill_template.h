#ifndef SKILL_TEMPLATE_H
#define SKILL_TEMPLATE_H

#include "skillbase.h"

class Skill_Template : public SkillBase
{
  private:
    void initialize(State st);
    Command control(State st);
    void spin();

};

#endif // SKILL_TEMPLATE_H
