#pragma once
#include <Eigen/Core>

namespace link4_sensor {
// Wrench ordering: [Fx Fy Fz Mx My Mz]. Moment remains about the sensor
// origin, matching the sensor-point Jacobian; do not add another p cross f.
inline Eigen::Matrix<double, 6, 1> rotateWrench(
        const Eigen::Matrix3d& rotation,
        const Eigen::Matrix<double, 6, 1>& sensorWrench, double sign) {
    Eigen::Matrix<double, 6, 1> result;
    result.head<3>() = sign * rotation * sensorWrench.head<3>();
    result.tail<3>() = sign * rotation * sensorWrench.tail<3>();
    return result;
}
} // namespace link4_sensor
