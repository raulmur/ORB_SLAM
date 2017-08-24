#include "Eigen/Core"
#include "Eigen/Dense"
#include <iostream>

void testeigeninverse()
{

    Eigen::Matrix<double,6,6> information = Eigen::Matrix<double,6,6>::Zero();
    information(5,5) = 1.0e8; information(0,0) = 1.0e8; information(1,1) = 1.0e8; information(2,2) = 1.0e8;

    std::cout << "information inverse "<< std::endl<< information.inverse() << std::endl;

}
int main(){
    testeigeninverse();
    return 0;
}
