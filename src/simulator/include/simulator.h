#ifndef INCL_SIMULATOR
#define INCL_SIMULATOR

#include <Eigen/Dense>
#include <numeric>
#include <cmath>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>

typedef struct{
    std::string name;
    std::function<Eigen::Matrix<double, Eigen::Dynamic, 1>(Eigen::Matrix<double, Eigen::Dynamic, 1>, Eigen::Matrix<double, Eigen::Dynamic, 1>)> dynamics;
    int state_num;
    int input_num;
}Model;

class Simulator{
public:
    Simulator(Model model, double integration_interval);
    void runSimulationThreadFunc();

    bool runSimulation(Eigen::Matrix<double, Eigen::Dynamic, 1> x0);
    bool setInput(Eigen::Matrix<double, Eigen::Dynamic, 1> u);
    bool stopSimulation();
    Eigen::Matrix<double, Eigen::Dynamic, 1> getState();
    
    
    bool is_running_ = false;
    Model model_;
private:
    Eigen::Matrix<double, Eigen::Dynamic, 1> doStep(std::function<Eigen::Matrix<double, Eigen::Dynamic, 1>(Eigen::Matrix<double, Eigen::Dynamic, 1>, Eigen::Matrix<double, Eigen::Dynamic, 1>)> f, Eigen::Matrix<double, Eigen::Dynamic, 1> x0, Eigen::Matrix<double, Eigen::Dynamic, 1> u);
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> ode4(std::function<Eigen::Matrix<double, Eigen::Dynamic, 1>(Eigen::Matrix<double, Eigen::Dynamic, 1>)> f, Eigen::Matrix<double, Eigen::Dynamic, 1> x0, double T);


    double integration_interval_ = 0.01;
    bool stop_sign_ = false;
    Eigen::Matrix<double, Eigen::Dynamic, 1> u_;
    Eigen::Matrix<double, Eigen::Dynamic, 1> x_;
   
    std::mutex input_lock_;
    std::mutex state_lock_;
};
void runSimulationThread(Simulator *sim);

#endif