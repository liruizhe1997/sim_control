#include "simulator.h"
#include <iostream>
#include "controller_base.h"

Eigen::Matrix<double, Eigen::Dynamic, 1> sys_2rd_dyn(Eigen::Matrix<double, Eigen::Dynamic, 1> x, Eigen::Matrix<double, Eigen::Dynamic, 1> u){
    Eigen::Matrix<double, Eigen::Dynamic, 1> dx{{x(1,0)},{x(0,0) + u(0,0)}};
    // std::cout << dx(0,0) << std::endl;
    return dx;
}

Model model = {"sys_2rd", sys_2rd_dyn, 2, 1};

int main(){
    Simulator sim(model, 0.00001);
    Controller controller(&sim);
    Eigen::Matrix<double, Eigen::Dynamic, 1> x0{{1},{0}};

    controller.runController();
    sim.runSimulation(x0);

    while(1){
        std::cin.get();
        if(controller.is_running_ == true){
            controller.stopController();
        }else{
            controller.runController();
        }
    }

    // std::cin.get();
    // controller.stopController();
    // std::cin.get();
    // sim.stopSimulation();
    // std::cin.get();
    // return 0;

}