#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <barrett/cdlbt/kinematics.h>
#include "link4_sensor_config.hpp"

namespace link4_sensor {
inline Eigen::Vector3d position(const bt_kinematics_link& link) {
    return Eigen::Vector3d(gsl_vector_get(link.origin_pos, 0),
                           gsl_vector_get(link.origin_pos, 1),
                           gsl_vector_get(link.origin_pos, 2));
}

// World axes, sensor origin, linear rows first. No mutation of shared kinematics.
template<int DOF>
void evaluate(const bt_kinematics& kin, const Mounting& mounting, Eigen::Matrix3d& rotation,
              Eigen::Vector3d& sensorPosition, Eigen::Matrix<double, 6, DOF>& jacobian) {
    static_assert(DOF >= 4, "Sensor requires at least four joints");
    Eigen::Matrix3d r4;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            r4(i,j) = gsl_matrix_get(kin.link[3]->rot_to_world, i, j);
    rotation = r4 * mounting.rotation;
    sensorPosition = position(*kin.link[3]) + r4 * mounting.position;
    jacobian.setZero();
    for (int j = 0; j < 4; ++j) {
        const bt_kinematics_link& previous = j == 0 ? *kin.base : *kin.link[j-1];
        Eigen::Vector3d axis;
        for (int i = 0; i < 3; ++i) axis[i] = gsl_vector_get(previous.axis_z, i);
        jacobian.template block<3,1>(0,j) = axis.cross(sensorPosition - position(previous));
        jacobian.template block<3,1>(3,j) = axis;
    }
}
} // namespace link4_sensor
