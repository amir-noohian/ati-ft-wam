#include "link4_wrench.hpp"
#include <cassert>
#include <iostream>

int main() {
    Eigen::Matrix3d r;
    r << 0,1,0, 0,0,1, 1,0,0;
    Eigen::Matrix<double,6,1> input;
    input << 1,2,3, 4,5,6;
    const auto rotated = link4_sensor::rotateWrench(r,input,1);
    Eigen::Matrix<double,6,1> expected;
    expected << 2,3,1, 5,6,4;
    assert((rotated-expected).norm() == 0);
    assert((link4_sensor::rotateWrench(r,input,-1)+expected).norm() == 0);
    // Pure moment must produce joint torque even with zero measured force.
    Eigen::Matrix<double,6,7> j = Eigen::Matrix<double,6,7>::Zero();
    j(1,0) = 0.2; // Linear velocity for a point offset 0.2 m along x.
    j(5,0) = 1.0; // Rotation about z.
    input << 0,0,0, 0,0,0.1;
    Eigen::Matrix<double,7,1> torque = j.transpose()*input;
    assert(std::abs(torque[0]-0.1)<1e-12);
    assert(torque.tail<3>().norm()==0);
    // Mixed wrench adds force lever arm and measured moment exactly once.
    input[1]=2.0;
    torque = j.transpose()*input;
    assert(std::abs(torque[0]-0.5)<1e-12);
    // Small force and moment contributions are retained without thresholds.
    input << 0,0.5,0, 0,0,0.01;
    torque = j.transpose()*input;
    assert(std::abs(torque[0]-0.11)<1e-12);
    std::cout << "Passed: wrench rotation/sign, small inputs, pure moment and combined wrench mapping.\n";
}
