#ifndef ROBOTBASE_H
#define ROBOTBASE_H

#include "taskbase.h"
#include "ros/ros.h"

class RobotBase
{
protected:
    TaskBase *task;

public:
    RobotBase(TaskBase *task);
    virtual ~RobotBase(){}
//    void advertise_state();
};



#endif // ROBOTBASE_H
