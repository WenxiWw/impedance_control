#include "ros/ros.h"
#include "robotbase.h"
#include "robot_sim.h"
#include "robot_real.h"

#include "tasks/test/task_test.h"
#include "tasks/pd_control/task_pd_control.h"
#include "tasks/cart_pd/task_cart_pd.h"


int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "panda_right");
  ros::Time::init();
  ROS_INFO("starting up");

  RobotBase *robot = nullptr;
  TaskBase *task = nullptr;

//  bool sim;
//  std::string path;
  std::string ip_address;
  std::string mujoco_path;

  if(argc < 3)
  {
    ROS_ERROR("Usage: %s <sub/dom/test> (sim \n or \n%s <sub/dom/test> <IP-Adress>)", argv[0], argv[0]);
    return 0;
  }
  else
  {
      if(std::strcmp(argv[1], "test") == 0)
      {
          task = new Task_Test;
          ROS_INFO("started Task Test");
      }
      else if(std::strcmp(argv[1], "pd") == 0)
      {
          task = new Task_PD_Control;
          ROS_INFO("started Task PD");
      }
      else if(std::strcmp(argv[1], "cart_pd") == 0)
      {
          task = new Task_Cart_PD;
          ROS_INFO("started Task Cart_Imp");
      }
      else
      {
          ROS_ERROR("task does not exist");
          return 0;
      }

    if(std::strcmp(argv[2], "sim") == 0)
    {
      ROS_INFO("Starting simulated robot");
      mujoco_path = argv[3];
      Robot_sim *robot_sim = new Robot_sim(mujoco_path, task);
      robot = robot_sim;
    }

    else
    {
      ROS_INFO("Starting real robot at IP %s", argv[2]);
      ip_address = argv[2];
      Robot_real *robot_real = new Robot_real(ip_address, task);
      robot = robot_real;
    }
  }

  ros::Rate rate(1000); // 1000 Hz as it is with real panda

  while(ros::ok())
  {
    rate.sleep();
    task->execute();
    ros::spinOnce();
  }

  delete robot;
  delete task;
  return 0;
}
