#include "simulator.h"
#include <iostream>

Eigen::Matrix<double, Eigen::Dynamic, 1> lorenz(Eigen::Matrix<double, Eigen::Dynamic, 1> x){
    Eigen::Matrix<double, Eigen::Dynamic, 1> dx{{10*(x(1,0)-x(0,0))},{24.74*x(0,0)-x(1,0)-x(0,0)*x(2,0)},{x(0,0)*x(1,0)-8.0/3.0*x(2,0)}};
    return dx;
}

Eigen::Matrix<double, Eigen::Dynamic, 1> sys_2rd(Eigen::Matrix<double, Eigen::Dynamic, 1> x){
    Eigen::Matrix<double, Eigen::Dynamic, 1> dx{{x(1,0)},{-x(0,0)}};
    return dx;
}

int main(){
    
    Simulator sim;
    // Eigen::Matrix<double, Eigen::Dynamic, 1> x0{{1},{1},{1}};
    Eigen::Matrix<double, Eigen::Dynamic, 1> x0{{1},{0}};
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> res_vec = sim.ode4(sys_2rd, x0, 10);
    Eigen::Matrix<double, Eigen::Dynamic, 1> res = res_vec.back();
    std::cout << "Integration result: " << res(0,0) << std::endl;
}