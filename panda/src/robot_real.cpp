#include "robot_real.h"

#include <array>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>

//franka::Robot *Robot_real::robot_static = NULL;
//franka::Model *Robot_real::model_static = NULL;
//Controller *Robot_real::ctrl_static = NULL;

void Robot_real::control_thread_function(const std::string ip_address)
{
    try {
        std::cout << "try to connect to robot with ip " << ip_address << std::endl;
        // connect to robot
        franka::Robot robot = franka::Robot(ip_address);
        //robot_static = robot;
        setDefaultBehavior(robot);
        // load the kinematics and dynamics model
        franka::Model model = franka::Model(robot.loadModel());
        model_real = new Model_real(&model);
        task->set_modelbase(model_real);

//        franka::Gripper gripper = franka::Gripper(ip_address);
//        gripper_real = new Gripper_real(&gripper);
//        task->set_gripperbase(gripper_real);


                //robot->loadModel();
        //this->model = &model;
        //model_static = model;

        //franka::RobotState robot_state = robot.readOnce();
        //State st;
        //st.f = robot_state;
        //for(int j=0; j<7; j++)
        //st.M = Eigen::Map<Eigen::Matrix<double,7,7>>(model.mass(robot_state).data()); //untested
        //this->ctrl = ctrl; //new Controller();
        //ctrl_static = ctrl;

        auto force_control_callback = [&](const franka::RobotState& robot_state,
                franka::Duration period) -> franka::Torques {

            //st.M = Eigen::Map<Eigen::Matrix<double,7,7>>(model.mass(robot_state).data());
            //st.bJ = Eigen::Map<Eigen::Matrix<double,6,7>>(model.bodyJacobian(franka::Frame::kEndEffector, robot_state).data());
            franka::Torques cmd = task->control(robot_state);

//            std::array<double, 7> tau_d_array{};
//            for(unsigned int j=0; j<7; j++)
//            {
//                tau_d_array[j] = cmd.tau[j];
//            }
            //      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_cmd;
            return cmd;
        };
        std::cout << "WARNING: Make sure sure that no endeffector is mounted and that the robot's last "
                     "joint is "
                     "in contact with a horizontal rigid surface before starting. Keep in mind that "
                     "collision thresholds are set to high values."
                  << std::endl
                  << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
        // start real-time control loop
        //control_thread_function = &franka::Robot::control;
        //control_thread = new std::thread(control_thread_function, robot, &Robot_real::control_callback, true, franka::kDefaultCutoffFrequency);

        //control_thread->join();
        robot.control(force_control_callback);


    } catch (const std::exception& ex) {
        // print exception
        std::cout << ex.what() << std::endl;

    }
}

Robot_real::Robot_real(const std::string ip_address, TaskBase *task) : RobotBase (task)
{

    control_thread = new std::thread(&Robot_real::control_thread_function, this, ip_address);

    //int main(int argc, char** argv) {
    //  // Check whether the required arguments were passed
    //  if (argc != 2) {
    //    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    //    return -1;
    //  }
    // parameters
    //  double desired_mass{0.0};
    //  constexpr double target_mass{1.0};    // NOLINT(readability-identifier-naming)
    //  constexpr double k_p{1.0};            // NOLINT(readability-identifier-naming)
    //  constexpr double k_i{2.0};            // NOLINT(readability-identifier-naming)
    //  constexpr double filter_gain{0.001};  // NOLINT(readability-identifier-naming)

    //control_thread->join();


}

Robot_real::~Robot_real()
{
    delete control_thread;
    delete model_real;
    delete gripper_real;
    //delete ctrl;
    //delete robot;
}

//franka::Torques Robot_real::control_callback(const franka::RobotState& robot_state, franka::Duration period)
//{
//    State st;
//    st.f = robot_state;
//    st.M = Eigen::Map<Eigen::Matrix<double,7,7>>(model_static->mass(robot_state).data());
//    st.bJ = Eigen::Map<Eigen::Matrix<double,6,7>>(model_static->bodyJacobian(franka::Frame::kEndEffector, robot_state).data());
//    Command cmd = ctrl_static->control(st);

//    std::array<double, 7> tau_d_array{};
//    for(unsigned int j=0; j<7; j++)
//    {
//        tau_d_array[j] = cmd.tau[j];
//    }
//    return tau_d_array;
//}

//State Robot_real::read_once()
//{
//    State st;
//    franka::RobotState robot_state = robot->readOnce();
//    for(int j=0; j<7; j++)
//    {
//        st.q[j] = robot_state.q[j];
//        st.dq[j] = robot_state.dq[j];
//    }
//    st.M = Eigen::Map<Eigen::Matrix<double,7,7>>(model->mass(robot_state).data());
//    return st;
//}

void Robot_real::setDefaultBehavior(franka::Robot& robot) {
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
}
