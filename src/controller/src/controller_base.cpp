#include "controller_base.h"


Controller::Controller(Simulator *sim){
    sim_ptr_ = sim;
}

bool Controller::runController(){
    std::thread controller_thread(runControllerThread, this);
    is_running_ = true;
    controller_thread.detach();
    return true;
}

void Controller::runControllerThreadFunc(){
    Eigen::Matrix<double, Eigen::Dynamic, 1> u;
    while(!stop_sign_) {
        if (sim_ptr_->is_running_ == true){
            u = getControlInput();
            sim_ptr_->setInput(u);
        }else{
        std::cout << "Simulator offline. Control failed!"<<std::endl;
        }
    }
    u = Eigen::MatrixXd::Zero(sim_ptr_->model_.input_num, 1);
    sim_ptr_->setInput(u);
    stop_sign_ = false;
    is_running_ = false;
}

// Eigen::Matrix<double, Eigen::Dynamic, 1> Controller::getControlInput(){
//     double Kp = 1;
//     Eigen::Matrix<double, Eigen::Dynamic, 1> x = sim_ptr_->getState();
//     Eigen::Matrix<double, Eigen::Dynamic, 1> u{{-Kp * x(1,0)}};
//     return u;
// }

Eigen::Matrix<double, Eigen::Dynamic, 1> Controller::getControlInput(){
    double Kp = 2.5;
    double Kd = 1.2;
    Eigen::Matrix<double, Eigen::Dynamic, 1> x = sim_ptr_->getState();
    Eigen::Matrix<double, Eigen::Dynamic, 1> u{{-Kp * x(0,0) - Kd * x(1,0)}};
    return u;
}

bool Controller::stopController(){
    stop_sign_ = true;
    return true;
}

void runControllerThread(Controller *controller){
    controller->is_running_ = true;
    controller->runControllerThreadFunc();
}