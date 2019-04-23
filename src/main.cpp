#include <iostream>
#include <Eigen/Dense>

#include <mujoco.h>

int main()
{
    Eigen::Matrix<mjtNum, Eigen::Dynamic, Eigen::Dynamic> goal;

    goal.resize(3,1);

    goal << 1, 0 ,0;


    return 0;
}