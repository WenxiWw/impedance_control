#ifndef TASKBASE_H
#define TASKBASE_H

#include "skillbase.h"
#include "gripperbase.h"

class TaskBase
{
private:
    virtual Skill statemachine(Skill skill, Flag flag) = 0;
    Skill current_skill = new Skill_0;
    std::mutex mtx;
    ModelBase *modelbase;
    GripperBase *gripperbase;

public:
  void set_modelbase(ModelBase *modelbase);
  void set_gripperbase(GripperBase *gripperbase);
  franka::Torques control(franka::RobotState state);
  virtual ~TaskBase();
  void execute();

};

#endif // TASKBASE_H
