#include "skills/skill_template.h"
#include "flags/flag.h"

void Skill_Template::initialize(State st)
{

}

Command Skill_Template::control(State st)
{

  return{0, 0, 0, 0, 0, 0, 0};
}

void Skill_Template::spin()
{
    if( false )
    {
        wave(terminated);
    }
}
