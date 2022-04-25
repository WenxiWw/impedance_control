#include "taskbase.h"

void TaskBase::execute()
{
    mtx.lock();
    current_skill->spin_wrap();

    if(current_skill->is_waving)
    {
        current_skill->reset_flag();
        current_skill = statemachine(current_skill, current_skill->flag);
        current_skill->set_modelbase(modelbase);
        current_skill->set_gripperbase(gripperbase);
    }
    mtx.unlock();
}

franka::Torques TaskBase::control(franka::RobotState state)
{

    mtx.lock();
    franka::Torques cmd = current_skill->control_wrap(state);
    mtx.unlock();
    return cmd;
}

TaskBase::~TaskBase()
{
}

void TaskBase::set_modelbase(ModelBase *modelbase)
{
    this->modelbase = modelbase;
}

void TaskBase::set_gripperbase(GripperBase *gripperbase)
{
    this->gripperbase = gripperbase;
}
