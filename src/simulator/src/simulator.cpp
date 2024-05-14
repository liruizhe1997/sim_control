#include "simulator.h"
#include <iostream>

Simulator::Simulator(Model model, double integration_interval): model_(model), 
                                                                integration_interval_(integration_interval){
    u_ = Eigen::MatrixXd::Zero(model.input_num,1);
    x_ = Eigen::MatrixXd::Zero(model.state_num,1);
    // x_(0,0) = 1;
};

std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> Simulator::ode4(std::function<Eigen::Matrix<double, Eigen::Dynamic, 1>(Eigen::Matrix<double, Eigen::Dynamic, 1>)> f, Eigen::Matrix<double, Eigen::Dynamic, 1> x0, double T) {
    int steps = (int)(T / integration_interval_);
    Eigen::Matrix<double, Eigen::Dynamic, 1> x = x0;
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> x_out;
    x_out.push_back(x);
    double h = T / steps;
    for(int i = 0; i < steps; ++i){
        Eigen::Matrix<double, Eigen::Dynamic, 1> K1 = f(x);
        Eigen::Matrix<double, Eigen::Dynamic, 1> K2 = f(x + h / 2 * K1);
        Eigen::Matrix<double, Eigen::Dynamic, 1> K3 = f(x + h / 2 * K2);
        Eigen::Matrix<double, Eigen::Dynamic, 1> K4 = f(x + h * K3);
        x = x + h / 6 * (K1 + 2 * K2 + 2 * K3 + K4);
        x_out.push_back(x);
        std::cout << x(0,0) << std::endl;
    }
    return x_out;
}

Eigen::Matrix<double, Eigen::Dynamic, 1> Simulator::doStep(std::function<Eigen::Matrix<double, Eigen::Dynamic, 1>(Eigen::Matrix<double, Eigen::Dynamic, 1>, Eigen::Matrix<double, Eigen::Dynamic, 1>)> f, Eigen::Matrix<double, Eigen::Dynamic, 1> x0, Eigen::Matrix<double, Eigen::Dynamic, 1> u){
    double h = integration_interval_;
    Eigen::Matrix<double, Eigen::Dynamic, 1> K1 = f(x0, u);
    Eigen::Matrix<double, Eigen::Dynamic, 1> K2 = f(x0 + h / 2.0 * K1, u);
    Eigen::Matrix<double, Eigen::Dynamic, 1> K3 = f(x0 + h / 2.0 * K2, u);
    Eigen::Matrix<double, Eigen::Dynamic, 1> K4 = f(x0 + h * K3, u);
    Eigen::Matrix<double, Eigen::Dynamic, 1> x1 = x0 + h / 6.0 * (K1 + 2.0 * K2 + 2.0 * K3 + K4);
    std::cout << "x:" << std::endl <<x1(0,0) <<std::endl;
    return x1;
}

bool Simulator::setInput(Eigen::Matrix<double, Eigen::Dynamic, 1> u){
    if(!is_running_){
        std::cout << "Simulation is not running. Set input failed!" << std::endl;
        return false;
    }
    input_lock_.lock();
    u_ = u;
    input_lock_.unlock();
    return true;
}

Eigen::Matrix<double, Eigen::Dynamic, 1> Simulator::getState(){
    state_lock_.lock();
    Eigen::Matrix<double, Eigen::Dynamic, 1> x = x_;
    state_lock_.unlock();
    return x;
}

bool Simulator::stopSimulation(){
    stop_sign_ = true;
    return true;
}

bool Simulator::runSimulation(Eigen::Matrix<double, Eigen::Dynamic, 1> x0){
    x_ = x0;
    std::thread sim_thread(runSimulationThread, this);
    is_running_ = true;
    sim_thread. detach();
    return true;
}

void runSimulationThread(Simulator *sim){
    sim->is_running_ = true;
    sim->runSimulationThreadFunc();
}

void Simulator::runSimulationThreadFunc(){
    while(!stop_sign_) {
        input_lock_.lock();
        Eigen::Matrix<double, Eigen::Dynamic, 1> u = u_;
        input_lock_.unlock();
        state_lock_.lock();
        x_ = doStep(model_.dynamics, x_, u);
        state_lock_.unlock();
        // std::cout << "fx:\n"<<model_.dynamics(x_,u_) << std::endl;
    }
    stop_sign_ = false;
    is_running_ = false;
}