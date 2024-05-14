#ifndef INCL_CONTROLLER
#define INCL_CONTROLLER

#include <Eigen/Dense>
#include <thread>
#include "simulator.h"
#include <iostream>


class Controller{
public:
    Controller(Simulator *sim);
    bool runController();
    void runControllerThreadFunc();
    bool stopController();


    bool is_running_ = false;

private:
    Eigen::Matrix<double, Eigen::Dynamic, 1> getControlInput();

    Simulator *sim_ptr_;
    bool stop_sign_ = false;
};

void runControllerThread(Controller *controller);

#endif