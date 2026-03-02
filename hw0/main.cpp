#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

int main(){
    Eigen::Vector3f v(2.0f, 1.0f, 1.0f);
    Eigen::Matrix3f rotate, tran;
    float a=sqrt(2)/2;
    rotate << a, (-1)*a, 0, a, a, 0, 0, 0, 1.0;
    tran << 1,0,1,0,1,2,0,0,1.0;
    std::cout << tran * rotate * v << std::endl;

    return 0;

}