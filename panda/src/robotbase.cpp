#include "robotbase.h"

RobotBase::RobotBase(TaskBase *task)
{
//    State st;
//    for(unsigned int j=0; j<7; j++)
//    {
//      st.f.q[j] = 0;
//      st.f.dq[j] = 0;
//    }
//    st.M.setZero();
  this->task = task;
}

//Robot_wrap::~Robot_wrap()
//{
//  //delete ctrl;
//}

//void Robot_wrap::advertise_state()
//{
//    ctrl->advertise_state();
//}


